"""AIR control hardware test: the 29 checks. conftest.py sets up the rig, CAN
and the pretend car; each test gets the car from one of its fixtures: `car`
(healthy, board not restarted yet) or `idle` (healthy, board restarted and
waiting in IDLE)."""
import time

import pytest


def test_startup(car):
    """4 s in INIT for the IMD to settle, then IDLE with both outputs off"""
    car.reset()
    time.sleep(3.5)
    assert car.state == "INIT", "left INIT before the IMD's 4 s"
    car.wait("IDLE", 2)
    car.sync()
    assert car.fault == "NONE" and not car.precharge_on and not car.main_on


# Break one thing, restart the board: it must refuse to start with this fault
STARTUP_FAULTS = {
    "bms_silent": ({"bms_on": False}, "CAN_BMS_TIMEOUT"),
    "ivt_silent": ({"ivt_on": False}, "CAN_GMETER_TIMEOUT"),
    "pack_under_200v": ({"pack_v": 100.0}, "BMS_VOLTAGE"),
    "bms_fault": ({"bms_fault": True}, "BMS_VOLTAGE"),
    "ivt_error": ({"ivt_error": True}, "IVT_STATUS"),
    "dc_link_charged": ({"volts": 10.0, "discharge_works": False}, "TRACTIVE_VOLTAGE"),
    "air_p_welded": ({"air_p": "closed"}, "AIR_P_WELD"),
    "air_n_welded": ({"air_n": "closed"}, "AIR_N_WELD"),
    "both_welded": ({"air_p": "closed", "air_n": "closed"}, "BOTH_AIRS_WELD"),
    "imd_fault": ({"imd_ok": False}, "IMD_STATUS"),
    "tsms_on": ({"open_nodes": set()}, "SHUTDOWN_IMPLAUSIBILITY"),
}


@pytest.mark.parametrize("name", STARTUP_FAULTS)
def test_startup_fault(car, name):
    changes, fault = STARTUP_FAULTS[name]
    car.set(**changes)
    car.reset()
    car.expect_fault(fault, timeout=6.5)


# Start in IDLE (or TS_ACTIVE if the first value is True), break one thing:
# the board must fault with this code, between the two times in seconds
RUNNING_FAULTS = {
    "idle_imd_fault": (False, {"imd_ok": False}, "IMD_STATUS", 0, 1),
    "idle_bms_silent": (False, {"bms_on": False}, "CAN_BMS_TIMEOUT", 0.8, 1.5),
    "idle_ivt_silent": (False, {"ivt_on": False}, "CAN_GMETER_TIMEOUT", 0.3, 1),
    "idle_dc_link_charged": (False, {"volts": 10.0, "discharge_works": False},
                             "TRACTIVE_VOLTAGE", 0, 1),
    "idle_air_n_welded": (False, {"air_n": "closed"}, "AIR_N_WELD", 0, 1),
    "tsms_on_hvd_open": (False, {"open_nodes": {"ss_hvd"}}, "SHUTDOWN_IMPLAUSIBILITY", 0, 1),
    "air_p_wont_close": (False, {"air_p": "open", "open_nodes": set()}, "CONTACTOR_FEEDBACK", 0, 3),
    "air_n_wont_close": (False, {"air_n": "open", "open_nodes": set()}, "CONTACTOR_FEEDBACK", 0, 3),
    "precharge_fail": (False, {"precharge_works": False, "open_nodes": set()},
                       "PRECHARGE_FAIL", 4.7, 5.6),
    "active_imd_fault": (True, {"imd_ok": False}, "IMD_STATUS", 0, 1),
    "active_bms_fault": (True, {"bms_fault": True}, "BMS_VOLTAGE", 0, 1),
    "active_air_n_drops_out": (True, {"air_n": "open"}, "CONTACTOR_FEEDBACK", 0, 1),
    "discharge_air_p_welded": (True, {"air_p": "closed", "open_nodes": {"ss_tsms"}},
                               "AIR_P_WELD", 0, 1),
    "discharge_air_n_welded": (True, {"air_n": "closed", "open_nodes": {"ss_tsms"}},
                               "AIR_N_WELD", 0, 1),
    "discharge_fail": (True, {"discharge_works": False, "open_nodes": {"ss_tsms"}},
                       "DISCHARGE_FAIL", 4.7, 5.6),  # FSAE EV.7.2.2.c: 60 V within 5 s
}


@pytest.mark.parametrize("name", RUNNING_FAULTS)
def test_running_fault(idle, name):
    from_active, changes, fault, earliest, latest = RUNNING_FAULTS[name]
    if from_active:
        idle.to_ts_active()
    idle.set(**changes)
    took = idle.expect_fault(fault, timeout=latest)
    assert took >= earliest, f"faulted after {took:.2f} s, before {earliest} s"


def test_drive_cycle_twice(idle):
    """IDLE -> PRECHARGE -> TS_ACTIVE -> DISCHARGE -> IDLE, twice in a row"""
    for _ in range(2):
        idle.set(open_nodes=set())  # the driver switches TSMS on
        idle.wait("PRECHARGE", 0.5)
        idle.sync()
        assert idle.precharge_on and not idle.main_on
        took = idle.wait("TS_ACTIVE", 5)
        assert took >= 0.5, f"closed our AIR after {took:.2f} s, before 95% precharge"
        time.sleep(0.3)
        idle.sync()
        assert idle.state == "TS_ACTIVE" and idle.main_on and not idle.precharge_on
        idle.set(open_nodes={"ss_tsms"})  # TSMS off
        idle.wait("DISCHARGE", 0.5)
        idle.wait("IDLE", 4)
        idle.sync()
        assert idle.fault == "NONE" and not idle.precharge_on and not idle.main_on


def test_node_opens_while_driving(idle):
    """A node before TSMS opens (like an e-stop): outputs off and the DC link
    drains, but the board waits in DISCHARGE until TSMS is off too"""
    idle.to_ts_active()
    idle.set(open_nodes={"ss_bms"})
    idle.wait("DISCHARGE", 0.5)
    time.sleep(3)
    assert idle.state == "DISCHARGE" and idle.volts < 5
    assert not idle.precharge_on and not idle.main_on
    idle.set(open_nodes={"ss_bms", "ss_tsms"})
    idle.wait("IDLE", 0.5)
