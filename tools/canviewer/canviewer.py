"""
FSAE CAN Viewer — Modern Dashboard for CANable 2 (gs_usb)
Parses mkvii.dbc and displays live decoded CAN signals in a
dark-themed, tabbed dashboard GUI.
"""

import tkinter as tk
from tkinter import ttk, messagebox, font as tkfont
import cantools
import can
import threading
import time
import os
import sys
import collections

# ── libusb / gs_usb bootstrap ──────────────────────────────────────
try:
    from gs_usb.gs_usb import GsUsb
    import usb.core
    import libusb_package

    _backend = libusb_package.get_libusb1_backend()
    _orig_find = usb.core.find

    def _patched_find(**kw):
        kw.setdefault("backend", _backend)
        return _orig_find(**kw)

    usb.core.find = _patched_find
except ImportError:
    GsUsb = None

# ── colour palette ──────────────────────────────────────────────────
BG_DARK      = "#0f1117"
BG_PANEL     = "#181b25"
BG_CARD      = "#1e2230"
BG_ENTRY     = "#262a38"
FG_PRIMARY   = "#e4e6f0"
FG_DIM       = "#7a7f95"
FG_ACCENT    = "#00d4aa"
FG_WARN      = "#ff6b6b"
FG_BLUE      = "#4ea8de"
FG_YELLOW    = "#f0c850"
BORDER       = "#2a2e3d"
HIGHLIGHT    = "#00d4aa"

# ── subsystem grouping ──────────────────────────────────────────────
SUBSYSTEM_MAP = {
    "Inverter": [
        "M160_Temperature_Set_1", "M161_Temperature_Set_2",
        "M162_Temperature_Set_3", "M163_Analog_Input_Voltages",
        "M164_Digital_Input_Status", "M165_Motor_Position_Info",
        "M166_Current_Info", "M167_Voltage_Info",
        "M168_Flux_ID_IQ_Info", "M169_Internal_Voltages",
        "M170_Internal_States", "M171_Fault_Codes",
        "M172_Torque_And_Timer_Info", "M173_Modulation_And_Flux_Info",
        "M174_Firmware_Info", "M175_Diag_Data_Message",
        "M176_Fast_Info", "M177_Torque_Capability",
        "M192_Command_Message", "M193_Read_Write_Param_Command",
        "M194_Read_Write_Param_Response",
    ],
    "BMS": [
        "bms_core", "bms_sense", "bms_metrics", "bms_debug",
        "bms_voltage", "bms_temperature", "bms_cooling", "bms_mux",
        "BMS_Current_Limit", "charging_cmd", "charging_fbk",
    ],
    "Safety": [
        "air_control_critical", "bspd", "dashboard",
    ],
    "Throttle": [
        "throttle", "throttle_debug",
    ],
    "Sensors": [
        "lpms_cu2_a", "lpms_cu2_b", "lpms_cu2_c", "lpms_cu2_d",
        "gps_coords", "wheel_speed", "sus_travel",
        "sus_strain_l", "sus_strain_r",
    ],
    "IVT": [
        "IVT_Msg_Result_I", "IVT_Msg_Result_U1", "IVT_Msg_Result_U2",
        "IVT_Msg_Result_U3", "IVT_Msg_Result_T", "IVT_Msg_Result_W",
        "IVT_Msg_Result_As", "IVT_Msg_Result_Wh",
        "Measurement", "Status",
    ],
    "Other": [
        "pdu_currents", "pdu_test", "can_print",
        "Reserved1", "Reserved2",
    ],
}


def _subsystem_for(msg_name: str) -> str:
    for sub, names in SUBSYSTEM_MAP.items():
        if msg_name in names:
            return sub
    return "Other"


# ── application ─────────────────────────────────────────────────────
class CanViewerApp:

    DBC_FILE = "mkvii.dbc"
    BITRATE = 500_000
    UI_REFRESH_MS = 80          # ~12 fps
    LOG_POLL_MS   = 500         # error-log tab refresh rate

    # ── error conditions monitored by the background logger ─────────
    # Each entry: msg (DBC message name), signal (signal name),
    #   mode ("rising_edge" fires on non-error→error transition,
    #         "monotonic"   fires when a counter increments),
    #   trigger (callable: value → bool, True = error state),
    #   label  (short identifier for the log),
    #   cause  (string or callable: value → string explaining likely cause).
    ERROR_CONDITIONS = [
        # ── BMS ──────────────────────────────────────────────────────
        {
            "msg": "bms_core", "signal": "bms_state",
            "mode": "rising_edge",
            "trigger": lambda v: v == 2,
            "label": "BMS FAULT STATE",
            "cause": (
                "The BMS state machine entered FAULT. Inspect bms_fault_code in the "
                "same message for a specific fault bit."
            ),
        },
        {
            "msg": "bms_core", "signal": "bms_fault_code",
            "mode": "rising_edge",
            "trigger": lambda v: isinstance(v, (int, float)) and int(v) != 0,
            "label": "BMS FAULT CODE",
            "cause": (
                "A non-zero BMS fault code was asserted. Each bit encodes a specific "
                "BMS fault — cross-reference with the BMS firmware fault_code enum. "
                "This typically accompanies a bms_state FAULT transition."
            ),
        },
        {
            "msg": "bms_core", "signal": "overcurrent_detect",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "BMS OVERCURRENT",
            "cause": (
                "BMS detected a pack overcurrent condition. Check pack_current and "
                "the IVT current measurement (IVT_Msg_Result_I)."
            ),
        },
        # ── AIR / Shutdown Circuit ───────────────────────────────────
        {
            "msg": "air_control_critical", "signal": "air_fault",
            "mode": "rising_edge",
            "trigger": lambda v: int(v) != 0,
            "label": "AIR FAULT",
            "cause": lambda v: {
                1:  "AIR- weld detected. The negative main contactor may be welded shut — inspect hardware immediately.",
                2:  "AIR+ weld detected. The positive main contactor may be welded shut — inspect hardware immediately.",
                3:  "Both AIRs welded.",
                4:  "Precharge failure.",
                5:  "Discharge failure.",
                6:  "Precharge failed with relay welded. Check AIR hardware and precharge circuit together.",
                7:  "CAN communication error on AIR controller. Check CAN bus wiring, termination, and node health.",
                8:  "BMS CAN timeout on AIR controller. BMS stopped sending air_control_critical — check BMS node.",
                9:  "G-meter/IMU CAN timeout on AIR controller. Check IMU node and harness.",
                10: "Shutdown circuit implausibility. Inconsistency in shutdown loop signals — check each ss_* signal.",
                11: "Tractive system voltage out of expected range. Check voltage sense wiring and HV bus.",
                12: "BMS voltage fault relayed to AIR controller. Investigate BMS fault codes and cell voltages.",
                13: "IMD (Isolation Monitoring Device) fault. Check battery isolation resistance and HV wiring integrity.",
            }.get(int(v), f"Unknown air_fault value {int(v)}. Refer to air_control firmware source."),
        },
        # ── Inverter ─────────────────────────────────────────────────
        {
            "msg": "M171_Fault_Codes", "signal": "Run_Fault_Lo",
            "mode": "rising_edge",
            "trigger": lambda v: isinstance(v, (int, float)) and int(v) != 0,
            "label": "INVERTER RUN FAULT (Lo)",
            "cause": (
                "Inverter run-time fault word (bits 0-15) is non-zero. Each bit maps "
                "to a specific fault. Common causes: motor or inverter overtemperature, "
                "motor overcurrent, low DC bus voltage, or resolver/encoder fault. "
                "Refer to the Rinehart PM100 manual Table 4 for bit definitions."
            ),
        },
        {
            "msg": "M171_Fault_Codes", "signal": "Run_Fault_Hi",
            "mode": "rising_edge",
            "trigger": lambda v: isinstance(v, (int, float)) and int(v) != 0,
            "label": "INVERTER RUN FAULT (Hi)",
            "cause": (
                "Inverter run-time fault word (bits 16-31) is non-zero. Common causes: "
                "gate driver fault, hardware over-limit trip, or control board fault. "
                "Refer to the Rinehart PM100 manual Table 4 for bit definitions."
            ),
        },
        {
            "msg": "M171_Fault_Codes", "signal": "Post_Fault_Lo",
            "mode": "rising_edge",
            "trigger": lambda v: isinstance(v, (int, float)) and int(v) != 0,
            "label": "INVERTER POST FAULT (Lo)",
            "cause": (
                "Inverter power-on self-test fault (bits 0-15). Fault detected at "
                "startup before motor operation. Possible causes: EEPROM checksum "
                "error, hardware self-check failure, or resolver calibration issue."
            ),
        },
        {
            "msg": "M171_Fault_Codes", "signal": "Post_Fault_Hi",
            "mode": "rising_edge",
            "trigger": lambda v: isinstance(v, (int, float)) and int(v) != 0,
            "label": "INVERTER POST FAULT (Hi)",
            "cause": (
                "Inverter power-on self-test fault (bits 16-31). Refer to the "
                "Rinehart PM100 manual Table 4 for bit definitions."
            ),
        },
        # ── IVT ──────────────────────────────────────────────────────
        *[
            {
                "msg": f"IVT_Msg_Result_{ch}",
                "signal": f"IVT_Result_{ch}_System_Error",
                "mode": "rising_edge",
                "trigger": lambda v: v == 1,
                "label": f"IVT SYSTEM ERROR ({ch})",
                "cause": (
                    f"IVT system error on measurement channel {ch}. Check IVT "
                    f"sensor power supply (12 V), CAN wiring, and termination. "
                    f"A system error indicates an internal IVT hardware fault."
                ),
            }
            for ch in ["I", "U1", "U2", "U3", "T", "W", "As", "Wh"]
        ],
        # ── BMS Communication Error Counters ─────────────────────────
        {
            "msg": "bms_metrics", "signal": "temperature_pec_error_count",
            "mode": "monotonic",
            "trigger": lambda v: isinstance(v, (int, float)) and v > 0,
            "label": "BMS TEMPERATURE PEC ERROR",
            "cause": (
                "BMS temperature measurement PEC (CRC) errors are incrementing. "
                "Indicates LTC6811 SPI communication faults on the thermistor "
                "daisy-chain. Check ribbon-cable connections to CSC boards, "
                "verify SPI isolator health, and inspect for EMI sources near harness."
            ),
        },
        {
            "msg": "bms_metrics", "signal": "voltage_pec_error_count",
            "mode": "monotonic",
            "trigger": lambda v: isinstance(v, (int, float)) and v > 0,
            "label": "BMS VOLTAGE PEC ERROR",
            "cause": (
                "BMS voltage measurement PEC (CRC) errors are incrementing. "
                "LTC6811 SPI communication fault on the cell-voltage daisy-chain. "
                "Check CSC board connectors, board seating, and harness routing."
            ),
        },
        {
            "msg": "bms_metrics", "signal": "i2c_error_count",
            "mode": "monotonic",
            "trigger": lambda v: isinstance(v, (int, float)) and v > 0,
            "label": "BMS I2C ERROR",
            "cause": (
                "BMS I2C error count is incrementing. I2C peripheral fault on the "
                "BMS board — check I2C devices (fuel gauge, external temp sensor), "
                "bus pull-up resistors, and for shorts on the I2C lines."
            ),
        },
        # ── Charger ───────────────────────────────────────────────────
        {
            "msg": "bms_debug", "signal": "charger_hardware_fault",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "CHARGER HARDWARE FAULT",
            "cause": (
                "Charger hardware fault (mirrored in bms_debug from charger). "
                "Check charger AC input voltage, output connection, and internal "
                "temperature. May indicate a blown fuse or internal charger fault."
            ),
        },
        {
            "msg": "bms_debug", "signal": "charger_communication",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "CHARGER COMMUNICATION FAULT",
            "cause": (
                "Charger CAN communication fault detected by BMS. Check the CAN "
                "connection to the charger and verify termination. Ensure the charger "
                "is powered and the correct baud rate is configured."
            ),
        },
        {
            "msg": "bms_debug", "signal": "charger_temperature_protection",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "CHARGER OVERTEMPERATURE",
            "cause": (
                "Charger over-temperature protection triggered. Allow the charger to "
                "cool before resuming. Check charger fan operation, ventilation, "
                "and ambient temperature in the charging area."
            ),
        },
        {
            "msg": "charging_fbk", "signal": "hardware_fault",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "CHARGER HARDWARE FAULT (fbk)",
            "cause": (
                "Charger hardware fault reported directly in charging_fbk message. "
                "Check charger AC input, DC output connections, and internal state."
            ),
        },
        {
            "msg": "charging_fbk", "signal": "communication_state",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "CHARGER COMMUNICATION FAULT (fbk)",
            "cause": (
                "Charger communication fault in charging_fbk. Check CAN bus wiring "
                "between charger and BMS, and verify correct termination."
            ),
        },
        # ── Throttle / APPS ──────────────────────────────────────────
        {
            "msg": "throttle_debug", "signal": "throttle_brake_implaus",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "THROTTLE/BRAKE IMPLAUSIBILITY",
            "cause": (
                "Both throttle (APPS) and brake (BSE) are simultaneously active beyond "
                "allowed thresholds (FSAE EV.5.7 implausibility rule). The throttle "
                "controller will cut motor torque until brake is released. Check throttle "
                "pedal travel, BSE wiring, sensor calibration, and for mechanical binding."
            ),
        },
        {
            "msg": "throttle_debug", "signal": "throttle_deviation",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "THROTTLE SENSOR DEVIATION",
            "cause": (
                "Dual APPS sensors disagree by more than the allowed tolerance "
                "(FSAE EV.5.6 requires <10% deviation). Check throttle potentiometer "
                "wiring, sensor supply voltage (5 V), connector seating, and mechanical "
                "pedal travel alignment between primary and secondary sensors."
            ),
        },
        {
            "msg": "throttle_debug", "signal": "throttle_r_out_of_range",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "THROTTLE-R OUT OF RANGE",
            "cause": (
                "Right (secondary) APPS sensor reading is outside the calibrated range. "
                "Check sensor wiring, 5 V supply rail, connector seating, and that the "
                "sensor has not been damaged or shifted mechanically."
            ),
        },
        {
            "msg": "throttle_debug", "signal": "throttle_l_out_of_range",
            "mode": "rising_edge",
            "trigger": lambda v: v == 1,
            "label": "THROTTLE-L OUT OF RANGE",
            "cause": (
                "Left (primary) APPS sensor reading is outside the calibrated range. "
                "Check sensor wiring, 5 V supply rail, connector seating, and that the "
                "sensor has not been damaged or shifted mechanically."
            ),
        },
    ]

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("FSAE CAN Viewer")
        self.root.geometry("1200x780")
        self.root.minsize(900, 600)
        self.root.configure(bg=BG_DARK)

        # ── fonts ───────────────────────────────────────────────────
        self._title_font  = tkfont.Font(family="Segoe UI", size=16, weight="bold")
        self._header_font = tkfont.Font(family="Segoe UI", size=11, weight="bold")
        self._label_font  = tkfont.Font(family="Segoe UI", size=10)
        self._small_font  = tkfont.Font(family="Segoe UI", size=9)
        self._mono_font   = tkfont.Font(family="Consolas", size=10)
        self._huge_font   = tkfont.Font(family="Segoe UI", size=32, weight="bold")

        # ── load DBC ────────────────────────────────────────────────
        if getattr(sys, 'frozen', False):
            base_path = sys._MEIPASS  # PyInstaller temp extraction dir
        else:
            base_path = os.path.dirname(os.path.abspath(__file__))
        dbc_path = os.path.join(base_path, self.DBC_FILE)
        try:
            self.db = cantools.database.load_file(dbc_path)
        except Exception as e:
            messagebox.showerror("DBC Error", f"Could not load {self.DBC_FILE}:\n{e}")
            root.destroy()
            return

        # ── state ───────────────────────────────────────────────────
        self.bus = None
        self.running = False
        self.signal_values: dict[tuple[str, str], object] = {}
        self.signal_nodes: dict[tuple[str, str], str] = {}   # tree iid
        self.msg_counter: dict[str, int] = {}
        self.total_frames = 0
        self.cell_voltages_grid: dict[int, list] = {}  # ic -> [volts...]

        # ── error log tab state ─────────────────────────────────────
        # Byte offset into the current log file; incremented as content is read
        # so the tab display only appends new lines rather than re-reading everything.
        self._error_log_file_pos: int = 0

        # ── error logger state ───────────────────────────────────────
        # Rolling buffer of the last 20 decoded messages (recv_time, name, decoded).
        # Written and read exclusively from the RX thread — no lock needed.
        self._msg_buffer: collections.deque = collections.deque(maxlen=20)
        # Tracks the last seen value for each monitored (msg, signal) pair so
        # we can detect rising-edge and monotonic transitions.
        self._error_prev_states: dict[tuple[str, str], object] = {}
        # Path to the active session error log file; None until first connect.
        self._error_log_path: str | None = None

        # ── build styles then UI ────────────────────────────────────
        self._build_styles()
        self._build_toolbar()
        self._build_status_bar()
        self._build_notebook()
        self._populate_trees()

        # auto-connect
        self.root.after(400, self._auto_connect)

    # ================================================================
    # STYLES
    # ================================================================
    def _build_styles(self):
        s = ttk.Style()
        s.theme_use("clam")

        # general
        s.configure(".", background=BG_DARK, foreground=FG_PRIMARY,
                     font=self._label_font)

        # notebook
        s.configure("Dark.TNotebook", background=BG_DARK,
                     borderwidth=0, tabmargins=[8, 6, 8, 2])
        s.configure("Dark.TNotebook.Tab",
                     background=BG_CARD, foreground=FG_DIM,
                     padding=[14, 6], font=self._header_font,
                     borderwidth=0)
        s.map("Dark.TNotebook.Tab",
              background=[("selected", BG_PANEL)],
              foreground=[("selected", FG_ACCENT)])

        # frames
        s.configure("Dark.TFrame", background=BG_DARK)
        s.configure("Card.TFrame", background=BG_PANEL)
        s.configure("Toolbar.TFrame", background=BG_PANEL)

        # labels
        s.configure("Title.TLabel",  background=BG_PANEL, foreground=FG_PRIMARY,
                     font=self._title_font)
        s.configure("Accent.TLabel", background=BG_PANEL, foreground=FG_ACCENT,
                     font=self._header_font)
        s.configure("Dim.TLabel",    background=BG_PANEL, foreground=FG_DIM,
                     font=self._small_font)
        s.configure("StatusOK.TLabel",  background=BG_PANEL, foreground=FG_ACCENT,
                     font=self._label_font)
        s.configure("StatusErr.TLabel", background=BG_PANEL, foreground=FG_WARN,
                     font=self._label_font)
        s.configure("DashVal.TLabel", background=BG_PANEL, foreground=FG_PRIMARY,
                     font=self._huge_font)

        # buttons
        s.configure("Accent.TButton",
                     background=FG_ACCENT, foreground=BG_DARK,
                     font=self._header_font, padding=[16, 6],
                     borderwidth=0)
        s.map("Accent.TButton",
              background=[("active", "#00b894"), ("disabled", BORDER)])

        s.configure("Flat.TButton",
                     background=BG_CARD, foreground=FG_DIM,
                     font=self._label_font, padding=[10, 4],
                     borderwidth=0)
        s.map("Flat.TButton",
              background=[("active", BG_ENTRY)],
              foreground=[("active", FG_PRIMARY)])

        # combobox
        s.configure("Dark.TCombobox",
                     fieldbackground=BG_ENTRY, background=BG_ENTRY,
                     foreground=FG_PRIMARY, arrowcolor=FG_DIM,
                     borderwidth=0, padding=4)

        # treeview — the centrepiece
        s.configure("Signal.Treeview",
                     background=BG_PANEL,
                     foreground=FG_PRIMARY,
                     fieldbackground=BG_PANEL,
                     rowheight=26,
                     font=self._mono_font,
                     borderwidth=0,
                     relief="flat")
        s.configure("Signal.Treeview.Heading",
                     background=BG_CARD,
                     foreground=FG_DIM,
                     font=self._small_font,
                     borderwidth=0,
                     relief="flat")
        s.map("Signal.Treeview",
              background=[("selected", "#1c3a4a")],
              foreground=[("selected", FG_ACCENT)])

        # scrollbar
        s.configure("Dark.Vertical.TScrollbar",
                     background=BG_CARD, troughcolor=BG_PANEL,
                     arrowcolor=FG_DIM, borderwidth=0)

    # ================================================================
    # TOOLBAR
    # ================================================================
    def _build_toolbar(self):
        bar = ttk.Frame(self.root, style="Toolbar.TFrame")
        bar.pack(fill=tk.X, padx=0, pady=0)

        inner = ttk.Frame(bar, style="Toolbar.TFrame")
        inner.pack(fill=tk.X, padx=16, pady=10)

        ttk.Label(inner, text="⚡ FSAE CAN Viewer", style="Title.TLabel"
                  ).pack(side=tk.LEFT)

        # right-side controls
        ctrl = ttk.Frame(inner, style="Toolbar.TFrame")
        ctrl.pack(side=tk.RIGHT)

        self.refresh_btn = ttk.Button(ctrl, text="↻ Refresh", style="Flat.TButton",
                                      command=self._refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 6))

        ttk.Label(ctrl, text="Device:", style="Dim.TLabel"
                  ).pack(side=tk.LEFT, padx=(0, 4))

        self.port_cb = ttk.Combobox(ctrl, values=self._scan_devices(),
                                    width=10, state="readonly",
                                    style="Dark.TCombobox")
        self.port_cb.pack(side=tk.LEFT, padx=(0, 8))
        if self.port_cb["values"]:
            self.port_cb.current(0)

        self.connect_btn = ttk.Button(ctrl, text="Connect",
                                       style="Accent.TButton",
                                       command=self._toggle)
        self.connect_btn.pack(side=tk.LEFT)

    # ================================================================
    # STATUS BAR
    # ================================================================
    def _build_status_bar(self):
        bar = ttk.Frame(self.root, style="Toolbar.TFrame")
        bar.pack(side=tk.BOTTOM, fill=tk.X, padx=0, pady=0)

        inner = ttk.Frame(bar, style="Toolbar.TFrame")
        inner.pack(fill=tk.X, padx=16, pady=6)

        self.status_lbl = ttk.Label(inner, text="● Disconnected",
                                     style="StatusErr.TLabel")
        self.status_lbl.pack(side=tk.LEFT)

        self.fps_lbl = ttk.Label(inner, text="", style="Dim.TLabel")
        self.fps_lbl.pack(side=tk.RIGHT)

        self.frame_lbl = ttk.Label(inner, text="Frames: 0", style="Dim.TLabel")
        self.frame_lbl.pack(side=tk.RIGHT, padx=(0, 20))

    # ================================================================
    # NOTEBOOK + TREEVIEWS
    # ================================================================
    def _build_notebook(self):
        self.notebook = ttk.Notebook(self.root, style="Dark.TNotebook")
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=12, pady=(4, 8))

        self.trees: dict[str, ttk.Treeview] = {}

        # 1. Dashboard Tab
        self.dash_frame = ttk.Frame(self.notebook, style="Dark.TFrame")
        self.notebook.add(self.dash_frame, text="  Dashboard  ")
        self._build_dashboard()

        # 2. Cell Voltages Tab
        self.volts_frame = ttk.Frame(self.notebook, style="Dark.TFrame")
        self.notebook.add(self.volts_frame, text="  Voltages  ")
        self._build_voltages_tab()

        # 3. Error Log Tab
        self.errlog_frame = ttk.Frame(self.notebook, style="Dark.TFrame")
        self.notebook.add(self.errlog_frame, text="  Error Log  ")
        self._build_error_log_tab()

        tab_order = ["Inverter", "BMS", "Safety", "Throttle",
                     "Sensors", "IVT", "Other"]

        for tab_name in tab_order:
            frm = ttk.Frame(self.notebook, style="Dark.TFrame")
            self.notebook.add(frm, text=f"  {tab_name}  ")

            tree = ttk.Treeview(
                frm,
                columns=("value", "unit", "updated"),
                show="tree headings",
                style="Signal.Treeview",
            )
            tree.heading("#0",      text="Message / Signal", anchor=tk.W)
            tree.heading("value",   text="Value",            anchor=tk.W)
            tree.heading("unit",    text="Unit",             anchor=tk.W)
            tree.heading("updated", text="Last Update",      anchor=tk.W)

            tree.column("#0",      width=320, minwidth=200)
            tree.column("value",   width=200, minwidth=100)
            tree.column("unit",    width=120, minwidth=60)
            tree.column("updated", width=120, minwidth=80)

            vsb = ttk.Scrollbar(frm, orient=tk.VERTICAL, command=tree.yview,
                                style="Dark.Vertical.TScrollbar")
            tree.configure(yscrollcommand=vsb.set)

            tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            vsb.pack(side=tk.RIGHT, fill=tk.Y)

            self.trees[tab_name] = tree

    def _build_dashboard(self):
        self.dash_vars = {}
        
        cards = [
            ("Pack Voltage", "V", "bms_core", "pack_voltage", "{:.1f}"),
            ("Pack Current", "A", "bms_core", "pack_current", "{:.1f}"),
            ("BMS State", "", "bms_core", "bms_state", "{}"),
            ("Max Cell Temp", "°C", "bms_sense", "max_temperature", "{:.1f}"),
            ("Min Cell Temp", "°C", "bms_sense", "min_temperature", "{:.1f}"),
            ("Motor Temp", "°C", "M161_Temperature_Set_2", "RTD1_Temperature", "{:.1f}"),
            ("Inverter Temp", "°C", "M160_Temperature_Set_1", "Module_A_Temp", "{:.1f}"),
            ("Max Cell V", "V", "_computed", "max_cell_voltage", "{:.4f}"),
            ("Min Cell V", "V", "_computed", "min_cell_voltage", "{:.4f}"),
        ]

        for i, (title, unit, msg, sig, fmt) in enumerate(cards):
            r = i // 3
            c = i % 3
            
            card = ttk.Frame(self.dash_frame, style="Card.TFrame")
            card.grid(row=r, column=c, padx=12, pady=12, sticky="nsew")
            self.dash_frame.columnconfigure(c, weight=1)
            self.dash_frame.rowconfigure(r, weight=1)
            
            ttk.Label(card, text=title, style="Accent.TLabel", justify="center").pack(pady=(20, 5))
            val_lbl = ttk.Label(card, text="--", style="DashVal.TLabel", justify="center")
            val_lbl.pack(pady=5)
            if unit:
                ttk.Label(card, text=unit, style="Dim.TLabel", justify="center").pack(pady=(0, 20))
            else:
                ttk.Label(card, text="", style="Dim.TLabel").pack(pady=(0, 20))
                
            self.dash_vars[(msg, sig)] = (val_lbl, fmt)

    def _build_voltages_tab(self):
        self.volt_canvas = tk.Canvas(self.volts_frame, bg=BG_PANEL, borderwidth=0, highlightthickness=0)
        vsb = ttk.Scrollbar(self.volts_frame, orient=tk.VERTICAL, command=self.volt_canvas.yview, style="Dark.Vertical.TScrollbar")
        self.volt_canvas.configure(yscrollcommand=vsb.set)
        
        self.volt_inner = ttk.Frame(self.volt_canvas, style="Card.TFrame")
        self.volt_canvas.create_window((0, 0), window=self.volt_inner, anchor="nw")
        
        self.volt_inner.bind("<Configure>", lambda e: self.volt_canvas.configure(scrollregion=self.volt_canvas.bbox("all")))
        
        self.volt_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        
        ttk.Label(self.volt_inner, text="IC", style="Accent.TLabel", width=6, anchor=tk.W).grid(row=0, column=0, padx=(10, 2), pady=8, sticky=tk.W)
        for i in range(17):
            ttk.Label(self.volt_inner, text=f"C{i+1}", style="Dim.TLabel", width=8, anchor=tk.E).grid(row=0, column=i+1, padx=2, pady=8, sticky=tk.E)

        self.ic_nodes = {}
        self.cell_labels = {}

    def _populate_trees(self):
        for msg in sorted(self.db.messages, key=lambda m: m.name):
            sub = _subsystem_for(msg.name)
            tree = self.trees.get(sub, self.trees["Other"])

            parent = tree.insert(
                "", tk.END,
                text=f"  {msg.name}  (0x{msg.frame_id:03X})",
                open=False,
            )

            for sig in msg.signals:
                unit = sig.unit or ""
                iid = tree.insert(
                    parent, tk.END,
                    text=f"    {sig.name}",
                    values=("—", unit, "—"),
                )
                self.signal_nodes[(msg.name, sig.name)] = (sub, iid)

    # ================================================================
    # ERROR LOG TAB
    # ================================================================
    def _build_error_log_tab(self):
        # ── top bar ──────────────────────────────────────────────────
        bar = ttk.Frame(self.errlog_frame, style="Toolbar.TFrame")
        bar.pack(fill=tk.X)

        self._errlog_path_lbl = ttk.Label(
            bar, text="No session active — connect to start logging.",
            style="Dim.TLabel",
        )
        self._errlog_path_lbl.pack(side=tk.LEFT, padx=12, pady=6)

        ttk.Button(
            bar, text="Clear Display", style="Flat.TButton",
            command=self._errlog_clear,
        ).pack(side=tk.RIGHT, padx=(0, 8), pady=4)

        # ── text area ────────────────────────────────────────────────
        text_frame = ttk.Frame(self.errlog_frame, style="Dark.TFrame")
        text_frame.pack(fill=tk.BOTH, expand=True)

        self.errlog_text = tk.Text(
            text_frame,
            bg=BG_PANEL, fg=FG_PRIMARY,
            font=self._mono_font,
            state=tk.DISABLED,
            wrap=tk.NONE,
            borderwidth=0, relief="flat",
            insertbackground=FG_PRIMARY,
            selectbackground="#1c3a4a", selectforeground=FG_ACCENT,
        )

        vsb = ttk.Scrollbar(text_frame, orient=tk.VERTICAL,
                            command=self.errlog_text.yview,
                            style="Dark.Vertical.TScrollbar")
        hsb = ttk.Scrollbar(text_frame, orient=tk.HORIZONTAL,
                            command=self.errlog_text.xview)
        self.errlog_text.configure(
            yscrollcommand=vsb.set, xscrollcommand=hsb.set,
        )
        vsb.pack(side=tk.RIGHT,  fill=tk.Y)
        hsb.pack(side=tk.BOTTOM, fill=tk.X)
        self.errlog_text.pack(fill=tk.BOTH, expand=True)

        # ── syntax-highlight tags ─────────────────────────────────────
        self.errlog_text.tag_configure("sep",     foreground=FG_DIM)
        self.errlog_text.tag_configure("header",  foreground=FG_WARN,
                                       font=self._header_font)
        self.errlog_text.tag_configure("meta",    foreground=FG_BLUE)
        self.errlog_text.tag_configure("section", foreground=FG_YELLOW)
        self.errlog_text.tag_configure("msg",     foreground=FG_ACCENT)
        self.errlog_text.tag_configure("cause",   foreground=FG_DIM)

    def _errlog_tag_for(self, line: str) -> str:
        """Return the Text tag name for a single log line."""
        if line.startswith("="):
            return "sep"
        if "ERROR EVENT" in line:
            return "header"
        if line.startswith(("Timestamp", "Message", "Signal", "Session", "Log file")):
            return "meta"
        if line.startswith(("Cause Analysis", "Last ")):
            return "section"
        if line.startswith("  ["):
            return "msg"
        if line.startswith("  ") and line.strip():
            return "cause"
        return ""

    def _errlog_clear(self):
        """Clear the display widget and rewind the read position to the start of
        the current log file so the next poll re-reads the full session."""
        self._error_log_file_pos = 0
        self.errlog_text.configure(state=tk.NORMAL)
        self.errlog_text.delete("1.0", tk.END)
        self.errlog_text.configure(state=tk.DISABLED)

    def _errlog_tick(self):
        """Scheduled callback: append any new log file content to the tab display."""
        if not self.running:
            return
        if self._error_log_path:
            try:
                with open(self._error_log_path, "r", encoding="utf-8") as fh:
                    fh.seek(self._error_log_file_pos)
                    new_content = fh.read()
                if new_content:
                    self._error_log_file_pos += len(new_content.encode("utf-8"))
                    at_bottom = self.errlog_text.yview()[1] >= 0.99
                    self.errlog_text.configure(state=tk.NORMAL)
                    for line in new_content.splitlines(keepends=True):
                        tag = self._errlog_tag_for(line)
                        self.errlog_text.insert(tk.END, line, tag)
                    self.errlog_text.configure(state=tk.DISABLED)
                    if at_bottom:
                        self.errlog_text.see(tk.END)
            except (OSError, UnicodeDecodeError):
                pass
        self.root.after(self.LOG_POLL_MS, self._errlog_tick)

    # ================================================================
    # DEVICE SCANNING
    # ================================================================
    def _scan_devices(self):
        if GsUsb is not None:
            devs = GsUsb.scan()
            if devs:
                return [f"gs_usb:{i}" for i in range(len(devs))]
        return []

    def _refresh_ports(self):
        devs = self._scan_devices()
        self.port_cb["values"] = devs
        if devs:
            self.port_cb.current(0)
        else:
            self.port_cb.set("")

    # ================================================================
    # CONNECT / DISCONNECT
    # ================================================================
    def _auto_connect(self):
        if self.port_cb.get():
            self._connect()

    def _toggle(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        sel = self.port_cb.get()
        if not sel:
            messagebox.showwarning("No device",
                                   "No CANable detected.\n"
                                   "Plug it in and click Refresh.")
            return

        try:
            idx = int(sel.split(":")[-1]) if ":" in sel else 0
            self.bus = can.interface.Bus(interface="gs_usb",
                                         channel=idx,
                                         bitrate=self.BITRATE)
            self.running = True
            self.total_frames = 0
            self._init_error_log()

            self.connect_btn.configure(text="Disconnect")
            self.port_cb.configure(state="disabled")
            self.status_lbl.configure(text=f"● Connected  ({sel} @ 500 kbps)",
                                      style="StatusOK.TLabel")

            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            self._ui_tick()
            self._errlog_tick()

        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            self._disconnect()

    def _disconnect(self):
        self.running = False
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

        self.connect_btn.configure(text="Connect")
        self.port_cb.configure(state="readonly")
        self.status_lbl.configure(text="● Disconnected", style="StatusErr.TLabel")

    # ================================================================
    # ERROR LOGGER
    # ================================================================
    def _init_error_log(self):
        """Create a timestamped error log file for this connection session."""
        if getattr(sys, "frozen", False):
            log_dir = os.path.dirname(sys.executable)
        else:
            log_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._error_log_path = os.path.join(log_dir, f"canviewer_errors_{timestamp}.log")
        self._error_prev_states.clear()
        self._msg_buffer.clear()
        self._error_log_file_pos = 0
        self.errlog_text.configure(state=tk.NORMAL)
        self.errlog_text.delete("1.0", tk.END)
        self.errlog_text.configure(state=tk.DISABLED)
        self._errlog_path_lbl.configure(
            text=f"Logging to: {self._error_log_path}"
        )
        with open(self._error_log_path, "w", encoding="utf-8") as fh:
            fh.write("FSAE CAN Viewer — Error Log\n")
            fh.write(f"Session started : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            fh.write(f"Log file        : {self._error_log_path}\n")
            fh.write("=" * 80 + "\n\n")

    def _check_errors(
        self,
        msg_name: str,
        decoded: dict,
        recv_time: float,
        buf_snapshot: list,
    ):
        """Evaluate decoded signals against ERROR_CONDITIONS; write log on new faults."""
        for cond in self.ERROR_CONDITIONS:
            if cond["msg"] != msg_name:
                continue
            sig = cond["signal"]
            if sig not in decoded:
                continue

            val = decoded[sig]
            key = (msg_name, sig)
            prev = self._error_prev_states.get(key)
            mode = cond.get("mode", "rising_edge")
            is_error = False

            try:
                if mode == "monotonic":
                    # Fire when the counter increments (skip very first observation
                    # to avoid false positives on reconnect with non-zero history).
                    if (
                        prev is not None
                        and isinstance(val, (int, float))
                        and isinstance(prev, (int, float))
                        and val > prev
                    ):
                        is_error = True
                else:  # rising_edge
                    currently_err = bool(cond["trigger"](val))
                    prev_err = bool(cond["trigger"](prev)) if prev is not None else False
                    is_error = currently_err and not prev_err
            except Exception:
                pass

            self._error_prev_states[key] = val

            if is_error:
                self._write_error_log(cond, msg_name, sig, val, recv_time, buf_snapshot)

    def _write_error_log(
        self,
        cond: dict,
        msg_name: str,
        sig: str,
        val: object,
        recv_time: float,
        buf_snapshot: list,
    ):
        """Append a formatted error event (with pre-error message history) to the log."""
        if not self._error_log_path:
            return

        cause = cond["cause"]
        cause_text: str = cause(val) if callable(cause) else cause

        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(recv_time))
        frac = f"{recv_time % 1:.3f}"[1:]   # ".123"

        lines: list[str] = []
        lines.append("=" * 80)
        lines.append(f"ERROR EVENT : {cond['label']}")
        lines.append(f"Timestamp   : {ts}{frac}")
        lines.append(f"Message     : {msg_name}")
        lines.append(f"Signal      : {sig}  =  {val}")
        lines.append("")
        lines.append("Cause Analysis:")
        # Wrap cause text at sentence boundaries for readability
        for sentence in cause_text.replace("\n", " ").split(". "):
            sentence = sentence.strip().rstrip(".")
            if sentence:
                lines.append(f"  {sentence}.")
        lines.append("")
        lines.append(f"Last {len(buf_snapshot)} CAN messages before this event:")
        if buf_snapshot:
            for idx, (buf_ts, buf_name, buf_decoded) in enumerate(buf_snapshot):
                buf_ts_str = (
                    time.strftime("%H:%M:%S", time.localtime(buf_ts))
                    + f"{buf_ts % 1:.3f}"[1:]
                )
                items = list(buf_decoded.items())
                decoded_str = "  ".join(f"{k}={v}" for k, v in items[:6])
                if len(items) > 6:
                    decoded_str += f"  (+{len(items) - 6} more)"
                lines.append(
                    f"  [{idx + 1:2d}]  {buf_ts_str}  {buf_name:<32s}  {decoded_str}"
                )
        else:
            lines.append("  (buffer empty — no prior messages captured)")
        lines.append("=" * 80)
        lines.append("")

        try:
            with open(self._error_log_path, "a", encoding="utf-8") as fh:
                fh.write("\n".join(lines) + "\n")
        except Exception:
            pass

    # ================================================================
    # RECEIVE THREAD
    # ================================================================
    def _rx_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                try:
                    db_msg = self.db.get_message_by_frame_id(msg.arbitration_id)
                    decoded = db_msg.decode(msg.data, decode_choices=False)
                    name = db_msg.name
                    self.total_frames += 1
                    self.msg_counter[name] = self.msg_counter.get(name, 0) + 1
                    for sig, val in decoded.items():
                        self.signal_values[(name, sig)] = val

                    # Capture snapshot of buffer *before* appending this message so
                    # the log shows the 20 messages that preceded the error event.
                    recv_ts = msg.timestamp if msg.timestamp else time.time()
                    buf_snapshot = list(self._msg_buffer)
                    self._msg_buffer.append((recv_ts, name, dict(decoded)))
                    self._check_errors(name, decoded, recv_ts, buf_snapshot)

                    # Custom multiplex logic for bms_voltage
                    if name == "bms_voltage":
                        ic = decoded.get("ic")
                        cell_reg = decoded.get("cell")
                        if ic is not None and cell_reg is not None:
                            if ic not in self.cell_voltages_grid:
                                self.cell_voltages_grid[ic] = ["—"] * 17
                            
                            c_idx = cell_reg * 3
                            if "voltage_1" in decoded and c_idx < 17:
                                self.cell_voltages_grid[ic][c_idx] = decoded["voltage_1"]
                            if "voltage_2" in decoded and c_idx + 1 < 17:
                                self.cell_voltages_grid[ic][c_idx + 1] = decoded["voltage_2"]
                            if "voltage_3" in decoded and c_idx + 2 < 17:
                                self.cell_voltages_grid[ic][c_idx + 2] = decoded["voltage_3"]
                except (KeyError, cantools.database.errors.DecodeError):
                    pass
            except Exception:
                if not self.running:
                    break

    # ================================================================
    # UI UPDATE TICK
    # ================================================================
    def _ui_tick(self):
        if not self.running:
            return

        ts = time.strftime("%H:%M:%S")
        snapshot = dict(self.signal_values)  # cheap copy

        for (msg_name, sig_name), val in snapshot.items():
            key = (msg_name, sig_name)

            # Update dashboard if needed
            if key in self.dash_vars:
                lbl, fmt = self.dash_vars[key]
                disp_val = val

                # Special conversion for BMS temperature (voltage to Celsius)
                if key in (("bms_sense", "max_temperature"), ("bms_sense", "min_temperature")) and isinstance(val, (int, float)):
                    # LTC6811 VREF2 = 3.0V; use bus value if available
                    vref = snapshot.get(("bms_sense", "current_vref"), 3.0)
                    if vref is None or vref <= 0.1:
                        vref = 3.0  # LTC6811 VREF2 standard

                    if 0 < val < vref:
                        import math
                        r_series = 10000.0
                        # NTC pull-up to VREF2: Vout = Vref * Rntc / (Rntc + Rseries)
                        r_ntc = r_series * val / (vref - val)
                        
                        if r_ntc > 0:
                            t_nom = 298.15
                            beta = 3950.0
                            t_k = 1.0 / ( (1.0/t_nom) + (1.0/beta)*math.log(r_ntc/10000.0) )
                            disp_val = t_k - 273.15
                    else:
                        disp_val = val  # Render the raw float instead of 0 if out of bounds

                if isinstance(disp_val, float):
                    lbl_txt = fmt.format(disp_val)
                elif key == ("bms_core", "bms_state"):
                    states = {0: "ACTIVE", 1: "CHARGING", 2: "FAULT"}
                    lbl_txt = states.get(val, str(val))
                else:
                    lbl_txt = str(val)
                lbl.configure(text=lbl_txt)

            if key not in self.signal_nodes:
                continue
            sub, iid = self.signal_nodes[key]
            tree = self.trees.get(sub)
            if tree is None:
                continue

            if isinstance(val, float):
                txt = f"{val:.4f}"
            else:
                txt = str(val)

            old = tree.item(iid, "values")
            tree.item(iid, values=(txt, old[1], ts))

        # Update the Cell Voltages grid
        all_cell_volts = []
        for ic, volts in self.cell_voltages_grid.items():
            for v in volts:
                if isinstance(v, float):
                    all_cell_volts.append(v)
                    
        max_cv = max(all_cell_volts) if all_cell_volts else None
        min_cv = min(all_cell_volts) if all_cell_volts else None

        for ic, volts in self.cell_voltages_grid.items():
            if ic not in self.ic_nodes:
                self.ic_nodes[ic] = True
                row = len(self.ic_nodes)
                ttk.Label(self.volt_inner, text=f" IC {ic}", style="Accent.TLabel", width=6, anchor=tk.W).grid(row=row, column=0, padx=(10, 2), pady=2, sticky=tk.W)
                for i in range(17):
                    lbl = tk.Label(self.volt_inner, text="—", font=self._mono_font, bg=BG_PANEL, fg=FG_PRIMARY, width=8, anchor=tk.E)
                    lbl.grid(row=row, column=i+1, padx=2, pady=2)
                    self.cell_labels[(ic, i)] = lbl

            for i, v in enumerate(volts):
                lbl = self.cell_labels.get((ic, i))
                if not lbl: continue
                
                if isinstance(v, float):
                    lbl.configure(text=f"{v:.4f}")
                    if v == max_cv and max_cv is not None:
                        lbl.configure(bg=FG_WARN, fg=BG_DARK)
                    elif v == min_cv and min_cv is not None:
                        lbl.configure(bg=FG_BLUE, fg=BG_DARK)
                    else:
                        lbl.configure(bg=BG_PANEL, fg=FG_PRIMARY)
                else:
                    lbl.configure(text=str(v), bg=BG_PANEL, fg=FG_DIM)

        # Update computed max/min cell voltage on dashboard
        if all_cell_volts:
            max_cv = max(all_cell_volts)
            min_cv = min(all_cell_volts)
            for comp_key, comp_val in [("max_cell_voltage", max_cv), ("min_cell_voltage", min_cv)]:
                dash_key = ("_computed", comp_key)
                if dash_key in self.dash_vars:
                    lbl, fmt = self.dash_vars[dash_key]
                    lbl.configure(text=fmt.format(comp_val))

        self.frame_lbl.configure(text=f"Frames: {self.total_frames:,}")

        self.root.after(self.UI_REFRESH_MS, self._ui_tick)

    # ================================================================
    # CLEANUP
    # ================================================================
    def shutdown(self):
        self._disconnect()
        self.root.destroy()


# ── entry point ─────────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = CanViewerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()
