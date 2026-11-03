"""Setup for the AIR control hardware test (air_hw_test.py): the settings, the
rig Arduino, the pretend car around the board, and the fixtures the tests use.
pytest loads this file by itself.

- A rig Arduino running rig_arduino/rig_arduino.ino flips the board's inputs
  and reads its two outputs. Check PINS below first. Its watchdog puts every
  input back to safe (open / not OK) if the laptop goes quiet for 1 s.
- A CAN adapter plays the BMS and IVT and reads the board's status:
  sudo ip link set can0 up type can bitrate 500000
- The ST-Link resets the board before each test, so leave it plugged in.
- Low voltage only: the pretend car stands in for the contactors, the precharge
  and discharge circuits and the accumulator.

Flash the board with its pin table filled in and AIR_BOARD_CONFIGURED = 1, then:
    pytest vehicle/mkviii/software/hv/air_control/tests/hardware
Set AIR_RIG_PORT or AIR_CAN_CHANNEL if they aren't /dev/ttyACM0 and can0.
"""
import math
import os
import subprocess
import threading
import time

import can
import pytest

PORT = os.environ.get("AIR_RIG_PORT", "/dev/ttyACM0")
CAN_CHANNEL = os.environ.get("AIR_CAN_CHANNEL", "can0")
CONTROLLED_NEGATIVE = True  # AIR_CONTROLLED_NEGATIVE in air_config.h: we switch AIR-
PACK_V = 400.0  # what the pretend BMS reports

# signal: (Arduino pin wired to it, True if high means closed / on / OK). The
# second value must match active_level in air_board_config.c. TIED: not wired;
# the board holds that node closed itself, so no test may open it.
# Set for the STM32G441 dev board: Arduino D2 to D9 go to EXT_SIG_1 to EXT_SIG_8.
# Flash it with --copt=-DAIR_BOARD_CONFIGURED=1
TIED = "tied"
PINS = {
    "precharge_ctl": (2, True), "main_ctl": (3, True),  # the board's outputs
    "ss_tsms": (4, False), "ss_hvd": (5, False), "ss_bms": (6, False),
    "imd_sense": (7, True), "air_p_feedback": (8, True), "air_n_feedback": (9, True),
    "ss_imd_latch": (TIED, False), "ss_mpc": (TIED, False), "ss_tsmp": (TIED, False),
    "ss_emeter": (TIED, False),
}
NODES = ["ss_imd_latch", "ss_mpc", "ss_tsmp", "ss_hvd", "ss_bms", "ss_emeter", "ss_tsms"]  # TSMS is last
STATES = ["INIT", "IDLE", "SHUTDOWN_CIRCUIT_CLOSED", "PRECHARGE", "TS_ACTIVE", "DISCHARGE", "FAULT"]
FAULTS = ["NONE", "AIR_N_WELD", "AIR_P_WELD", "BOTH_AIRS_WELD", "PRECHARGE_FAIL",
          "DISCHARGE_FAIL", "PRECHARGE_FAIL_RELAY_WELDED", "CAN_ERROR", "CAN_BMS_TIMEOUT",
          "CAN_GMETER_TIMEOUT", "SHUTDOWN_IMPLAUSIBILITY", "TRACTIVE_VOLTAGE", "BMS_VOLTAGE",
          "IMD_STATUS", "BOARD_CONFIG", "CONTACTOR_FEEDBACK", "IVT_STATUS"]  # air.h order


class Rig:
    """The rig Arduino. set(name, True) closes a node or makes a signal OK;
    get(name) is True while the board switches that output on"""

    def __init__(self, port):
        import serial

        missing = [name for name, (pin, _) in PINS.items() if pin is None]
        if missing:
            raise RuntimeError("Fill in the Arduino pins in PINS: " + ", ".join(missing))
        self.serial = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(2)  # opening the port restarts most Arduinos
        self.serial.reset_input_buffer()
        # A rig whose watchdog tripped after the last run says so, once
        self._send(f"r {PINS['precharge_ctl'][0]}", ("0", "1", "tripped"))
        for name in ["precharge_ctl", "main_ctl"]:
            self._send(f"i {PINS[name][0]}")
        for name in NODES + ["imd_sense", "air_p_feedback", "air_n_feedback"]:
            pin, high = PINS[name]
            if pin != TIED:
                self._send(f"s {pin} {int(not high)}")  # safe for the watchdog: open / not OK
                self.set(name, False)  # level first, then output, so it never glitches
                self._send(f"o {pin}")

    def set(self, name, active):
        pin, high = PINS[name]
        if pin == TIED:
            assert active, f"{name} isn't wired to the rig, so it can't be opened"
            return
        self._send(f"w {pin} {int(active == high)}")

    def get(self, name):
        pin, high = PINS[name]
        return (self._send(f"r {pin}", ("0", "1")) == "1") == high

    def reset_board(self):
        subprocess.run(["openocd", "-f", "interface/stlink.cfg", "-f", "target/stm32g4x.cfg",
                        "-c", "init; reset run; shutdown"], check=True, capture_output=True)

    def close(self):
        self.serial.close()

    def _send(self, line, replies=("ok",)):
        """Send one command; its one-line answer must be one of `replies`"""
        self.serial.write(line.encode() + b"\n")
        reply = self.serial.readline().decode().strip()
        if reply == "tripped" and "tripped" not in replies:
            raise RuntimeError("The rig's watchdog let go of the board: the laptop sent "
                               "it nothing for over 1 s")
        if reply not in replies:
            raise RuntimeError(f"Rig Arduino answered {line!r} with {reply!r}")
        return reply


class Car:
    """Everything around the board. A test changes the settings in healthy(),
    and a background loop makes the rig and CAN match every 5 ms: it sends the
    BMS and IVT messages, closes each AIR's feedback when its coil is powered,
    and moves the DC link voltage through precharge and discharge"""

    def __init__(self, rig, bus):
        self.rig, self.bus = rig, bus
        self.lock = threading.Lock()
        self.healthy()
        self.state = self.fault = None  # from the board's last status message
        self.precharge_on = self.main_on = False  # the board's outputs
        self.written, self.passes = {}, 0
        self.error = None  # what stopped the background loop, if anything
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def healthy(self):
        """Every part working, the driver's switch (TSMS) off"""
        with self.lock:
            self.open_nodes = {"ss_tsms"}  # shutdown nodes that are open
            self.imd_ok = True
            self.bms_on = self.ivt_on = True  # sending on CAN
            self.pack_v = PACK_V
            self.bms_fault = self.ivt_error = False
            self.air_p = self.air_n = "auto"  # follow the coil, stuck "open" or welded "closed"
            self.precharge_works = self.discharge_works = True
            self.volts = 0.0  # DC link voltage the IVT reports

    def set(self, **changes):
        with self.lock:
            for name, value in changes.items():
                assert hasattr(self, name), name
                setattr(self, name, value)

    def check(self):
        """Raise whatever stopped the background loop, so a test fails instead of hanging"""
        if self.error is not None:
            raise RuntimeError(f"The pretend car stopped: {self.error}") from self.error

    def sync(self):
        """Wait for two full passes of the loop, so the rig matches the settings"""
        start, deadline = self.passes, time.monotonic() + 2
        while self.passes < start + 2:
            self.check()
            if time.monotonic() > deadline:
                raise RuntimeError("The pretend car stopped updating the rig")
            time.sleep(0.002)

    def reset(self):
        """Restart the board and forget its old status"""
        self.sync()
        self.rig.reset_board()
        time.sleep(0.1)  # a status sent just before the reset arrives first
        self.state = self.fault = None

    def wait(self, state, timeout):
        """Wait for the board to report this state; returns the seconds it took"""
        start = time.monotonic()
        while self.state != state:
            self.check()
            if time.monotonic() - start > timeout:
                raise AssertionError(
                    f"Expected {state} within {timeout} s; board says {self.state}, fault {self.fault}"
                    if self.state else "No status from the board: is it powered and flashed, "
                    "with AIR_BOARD_CONFIGURED = 1, and is CAN wired?")
            time.sleep(0.005)
        return time.monotonic() - start

    def expect_fault(self, fault, timeout=1.0):
        took = self.wait("FAULT", timeout)
        assert self.fault == fault
        self.sync()
        assert not self.precharge_on and not self.main_on, "FAULT with an output still on"
        return took

    def to_ts_active(self):
        self.set(open_nodes=set())
        self.wait("TS_ACTIVE", 3)
        time.sleep(0.3)  # past the 200 ms our AIR has to read closed
        self.sync()
        assert self.state == "TS_ACTIVE" and self.main_on and not self.precharge_on

    def _run(self):
        last = last_can = time.monotonic()
        try:
            while not self.stop.is_set():
                time.sleep(0.005)
                with self.lock:
                    now = time.monotonic()
                    self._listen()
                    self._step(now - last)
                    if now - last_can >= 0.02:
                        self._send_can()
                        last_can = now
                    last = now
                self.passes += 1
        except Exception as error:  # the rig or CAN failed: stop, and let check() report it
            self.error = error

    def _listen(self):
        while (frame := self.bus.recv(0)) is not None:
            if frame.arbitration_id == 0x00D and len(frame.data) == 4:
                fault, state = frame.data[0], frame.data[1]
                self.fault = FAULTS[fault] if fault < len(FAULTS) else fault
                self.state = STATES[state] if state < len(STATES) else state

    def _step(self, dt):
        self.precharge_on = self.rig.get("precharge_ctl")
        self.main_on = self.rig.get("main_ctl")
        loop_powered = not self.open_nodes
        coil = {"air_p": loop_powered, "air_n": self.main_on}
        if not CONTROLLED_NEGATIVE:
            coil = {"air_p": self.main_on, "air_n": loop_powered}
        closed = {air: {"auto": on, "open": False, "closed": True}[getattr(self, air)]
                  for air, on in coil.items()}
        # Write only changes, nodes first with TSMS last, then the IMD and AIRs
        want = {node: node not in self.open_nodes for node in NODES}
        want.update(imd_sense=self.imd_ok, air_p_feedback=closed["air_p"],
                    air_n_feedback=closed["air_n"])
        for name, active in want.items():
            if self.written.get(name) != active:
                self.rig.set(name, active)
                self.written[name] = active

        # The DC link voltage heads for whatever the circuit connects it to
        loop_air = closed["air_p" if CONTROLLED_NEGATIVE else "air_n"]
        if closed["air_p"] and closed["air_n"]:
            target, tau = self.pack_v, 0.01
        elif loop_air and self.precharge_on and self.precharge_works:
            target, tau = self.pack_v, 0.3  # about 0.9 s to 95%
        elif not loop_powered and self.discharge_works:
            target, tau = 0.0, 0.4  # about 1.8 s to under 5 V
        else:
            return  # nothing connected: it holds
        self.volts += (target - self.volts) * (1 - math.exp(-dt / tau))

    def _send_can(self):
        frames = []
        if self.bms_on:  # state FAULT = 2 at bit 0, 0.0256 V/count at bit 18
            bits = (2 if self.bms_fault else 0) | round(self.pack_v / 0.0256) << 18
            frames.append((0x010, bits.to_bytes(7, "little")))
        if self.ivt_on:  # result U1, error flags, signed millivolts
            frames.append((0x414, bytes([1, 0x80 if self.ivt_error else 0]) +
                           round(self.volts * 1000).to_bytes(4, "big", signed=True)))
        for frame_id, data in frames:
            try:
                self.bus.send(can.Message(arbitration_id=frame_id, data=data, is_extended_id=False))
            except can.CanError:
                pass  # nothing acknowledging on the bus; the test reports the silent board


def open_rig():
    return Rig(PORT)


def open_bus():
    return can.Bus(interface="socketcan", channel=CAN_CHANNEL, bitrate=500000)


@pytest.fixture(scope="session")
def bench():
    rig, bus = open_rig(), open_bus()
    car = Car(rig, bus)
    yield car
    car.stop.set()
    car.thread.join()
    bus.shutdown()
    rig.close()


@pytest.fixture
def car(bench):
    """A healthy car; the test restarts the board when it's ready"""
    bench.healthy()
    return bench


@pytest.fixture
def idle(car):
    """A healthy car with the board restarted and waiting in IDLE"""
    car.reset()
    car.wait("IDLE", 6.5)
    assert car.fault == "NONE"
    return car
