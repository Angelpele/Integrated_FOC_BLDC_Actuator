"""
//===================================================================================================//
//                                            Motor Monitor                                          //
//===================================================================================================//
"""

import sys, re, collections, csv, time, serial, serial.tools.list_ports
from datetime import datetime

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QLabel, QPushButton, QComboBox, QSpinBox,
    QGroupBox, QGridLayout, QTextEdit, QFileDialog, QSplitter,
    QStatusBar, QFrame, QCheckBox, QSlider, QMessageBox, QStyle
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QColor, QPalette, QIcon

import pyqtgraph as pg
import numpy as np

pg.setConfigOption('background', '#0e1117')
pg.setConfigOption('foreground', '#c8cdd8')
pg.setConfigOption('antialias', True)

DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #0e1117;
    color: #c8cdd8;
    font-family: 'Segoe UI', 'Inter', sans-serif;
    font-size: 13px;
}
QTabWidget::pane { border: 1px solid #1f2535; background: #0e1117; }
QTabBar::tab {
    background: #161b27; color: #7a8399; padding: 8px 20px;
    border: 1px solid #1f2535; border-bottom: none;
    border-radius: 4px 4px 0 0; margin-right: 2px; font-weight: 500;
}
QTabBar::tab:selected { background: #0e1117; color: #00d4aa; border-bottom: 2px solid #00d4aa; }
QTabBar::tab:hover:!selected { background: #1a2030; }
QPushButton {
    background: #1a2030; color: #c8cdd8; border: 1px solid #2a3348;
    border-radius: 6px; padding: 7px 16px; font-weight: 500;
}
QPushButton:hover { background: #1f2840; border-color: #3a4560; }
QPushButton:pressed { background: #151c2e; }
QPushButton#btn_connect { background: #0d3d2e; color: #00d4aa; border-color: #00d4aa44; }
QPushButton#btn_connect:hover { background: #0f4a38; }
QPushButton#btn_disconnect { background: #3d1220; color: #ff4d6d; border-color: #ff4d6d44; }
QPushButton#btn_record { background: #3d2000; color: #ffb347; border-color: #ffb34744; }
QPushButton#btn_brake { background: #3d0d0d; color: #ff4d6d; border-color: #ff4d6d55; font-weight: 700; }
QPushButton#btn_brake:hover { background: #5c1010; }
QComboBox, QSpinBox {
    background: #161b27; border: 1px solid #2a3348;
    border-radius: 5px; padding: 5px 10px; color: #c8cdd8;
}
QComboBox::drop-down { border: none; width: 22px; }
QComboBox:hover, QSpinBox:hover { border-color: #3a4560; }
QGroupBox {
    border: 1px solid #1f2535; border-radius: 8px;
    margin-top: 12px; padding-top: 8px; font-weight: 600; color: #7a8399;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
QTextEdit {
    background: #080b10; border: 1px solid #1f2535; border-radius: 6px;
    color: #7aff99; font-family: 'Consoles', monospace; font-size: 12px; padding: 6px;
}
QLabel#metric_val  { font-size: 22px; font-weight: 700; color: #c8cdd8; }
QLabel#metric_unit { font-size: 11px; color: #5a6070; }
QLabel#metric_name { font-size: 11px; color: #7a8399; }
QStatusBar { background: #080b10; border-top: 1px solid #1f2535; }
QCheckBox::indicator {
    width: 16px; height: 16px; border: 1px solid #2a3348;
    border-radius: 3px; background: #161b27;
}
QCheckBox::indicator:checked { background: #00d4aa; border-color: #00d4aa; }
"""

# CSV format from firmware:
# ID,pvdd,i1,i2,i3,a,b,c,nfault,noctw,ntc,m_ang,red_ang,m_rpm,red_rpm,state
# e.g.: 1,14.121,0.014,0.010,-0.024,27.12,0.000,0.000,0.00,0.00,0,0,0,1,1,0

STATE_NAMES = {0:"IDLE", 1:"ALIGNING", 2:"RAMPING", 3:"SPINNING", 4:"RAMP_DOWN", 5:"BRAKING"}
PLOT_LEN    = 500
state_str = "—"

# ─────────────────────────────────────────────────────────────
#  SERIAL WORKER
# ─────────────────────────────────────────────────────────────
class SerialWorker(QThread):
    data_ready = pyqtSignal(dict)
    raw_line   = pyqtSignal(str)
    error      = pyqtSignal(str)   # recoverable: log only, keep running
    fatal      = pyqtSignal(str)   # port lost: triggers disconnect

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self.ser= None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.fatal.emit(f"Cannot open {self.port}: {e}")
            return

        buf = b""
        consecutive_errors = 0

        while self._running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue

                consecutive_errors = 0
                buf += chunk

                while b"\n" in buf:
                    line_bytes, buf = buf.split(b"\n", 1)
                    line = line_bytes.decode('utf-8', errors='replace').strip()
                    if not line:
                        continue

                    self.raw_line.emit(line)

                    parts = line.split(',')
                    if len(parts) == 15:
                        try:
                            self.data_ready.emit({
                                'ID':      int(parts[0]),
                                'pvdd':    float(parts[1]),
                                'i1':      float(parts[2]),
                                'i2':      float(parts[3]),
                                'i3':      float(parts[4]),
                                'a':   float(parts[5]),
                                'b':   float(parts[6]),
                                'c':   float(parts[7]),
                                'nfault':  int(parts[8]),
                                'noctw':   int(parts[9]),
                                'ntc':     float(parts[10]),
                                'm_ang':   float(parts[11]),
                                'red_ang': float(parts[12]),
                                'm_rpm':   float(parts[13]),
                                'red_rpm': float(parts[14]),
                                'ts':      time.time(),
                            })
                        except (ValueError, IndexError) as e:
                            self.error.emit(f"Parse: {e} | {line[:60]}")

            except serial.SerialException as e:
                self.fatal.emit(f"Port lost: {e}")
                break

            except Exception as e:
                consecutive_errors += 1
                self.error.emit(f"Error #{consecutive_errors}: {e}")
                if consecutive_errors >= 20:
                    self.fatal.emit(f"Too many errors: {e}")
                    break

        try:
            self.ser.close()
        except Exception:
            pass

    def stop(self):
        self._running = False
        self.wait()

    def write_data(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data.encode('utf-8'))
            except Exception as e:
                self.error.emit(f"Write failed: {e}")

# ─────────────────────────────────────────────────────────────
#  METRIC CARD
# ─────────────────────────────────────────────────────────────
class MetricCard(QFrame):
    def __init__(self, name, unit="", warn=None, danger=None, high_is_bad=True):
        super().__init__()
        self.warn = warn; self.danger = danger; self.high_is_bad = high_is_bad
        self.setObjectName("metric_card")
        self.setStyleSheet("""
            QFrame#metric_card {
                background: #161b27; border: 1px solid #1f2535;
                border-radius: 10px; padding: 6px;
            }
        """)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(2)
        self.lbl_name = QLabel(name.upper()); self.lbl_name.setObjectName("metric_name")
        self.lbl_val  = QLabel("—");          self.lbl_val.setObjectName("metric_val")
        self.lbl_unit = QLabel(unit);         self.lbl_unit.setObjectName("metric_unit")
        lay.addWidget(self.lbl_name)
        lay.addWidget(self.lbl_val)
        lay.addWidget(self.lbl_unit)

    def update(self, val, fmt=".2f"):
        self.lbl_val.setText(f"{val:{fmt}}")
        if self.danger is not None and (val >= self.danger if self.high_is_bad else val <= self.danger):
            color = "#ff4d6d"
        elif self.warn is not None and (val >= self.warn if self.high_is_bad else val <= self.warn):
            color = "#ffb347"
        else:
            color = "#00d4aa"
        self.lbl_val.setStyleSheet(f"font-size:22px; font-weight:700; color:{color};")

# ─────────────────────────────────────────────────────────────
#  PLOT HELPER
# ─────────────────────────────────────────────────────────────
def make_plot(title, ylabel, colors):
    pw = pg.PlotWidget()
    pw.setTitle(title, color='#7a8399', size='11pt')
    pw.setLabel('left', ylabel, color='#5a6070')
    pw.showGrid(x=True, y=True, alpha=0.15)
    pw.getAxis('bottom').setStyle(tickFont=QFont('Consoles', 9))
    pw.getAxis('left').setStyle(tickFont=QFont('Consoles', 9))
    pw.setBackground('#0e1117')
    return pw, [pw.plot(pen=pg.mkPen(color=c, width=1.8)) for c in colors]

# ─────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────
class MotorMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Monitor  //  ESP32-S3 Controller")
        self.resize(1280, 820)
        self.worker = None
        self.recording = False
        self.rec_file = self.rec_writer = None

        N = PLOT_LEN
        self.buf = {k: collections.deque([0.0]*N, maxlen=N) for k in
                    ['pvdd','i1','i2','i3','m_rpm','red_rpm','ntc','m_ang','a','b','c']}
        self.x = np.arange(N)

        self._build_ui()
        self.setStyleSheet(DARK_STYLE)

        self._plot_timer = QTimer()
        self._plot_timer.setInterval(20)
        self._plot_timer.timeout.connect(self._refresh_plots)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_toolbar())

        tabs = QTabWidget()
        tabs.setDocumentMode(True)
        tabs.addTab(self._tab_dashboard(),  "  Dashboard  ")
        tabs.addTab(self._tab_control(),    "  Control  ")
        #tabs.addTab(self._tab_gyro(), "  Gyro - Accel  ")
        tabs.addTab(self._tab_currents(),   "  Currents  ")
        tabs.addTab(self._tab_mechanical(), "  Mechanical  ")
        tabs.addTab(self._tab_power(),      "  Power - NTC  ")
        tabs.addTab(self._tab_console(),    "  Raw Console  ")
        root.addWidget(tabs)

        self.status = QStatusBar()
        self.setStatusBar(self.status) 
        self._status("No Connection", "warn")

    def _build_toolbar(self):
        bar = QWidget()
        bar.setStyleSheet("background:#080b10; border-bottom:1px solid #1a2030;")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 8, 16, 8)
        lay.setSpacing(10)

        logo = QLabel("ASTURION")
        logo.setStyleSheet("font-family:'Consoles'; font-size:14px; font-weight:700; color:#00d4aa; letter-spacing:3px;")
        lay.addWidget(logo)
        lay.addWidget(QLabel("//"))

        lay.addWidget(QLabel("Port:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(140)
        lay.addWidget(self.cb_port)

        btn_ref = QPushButton("")
        btn_ref.setFixedSize(34, 32)
        btn_ref.setIcon(QApplication.style().standardIcon(QStyle.StandardPixmap.SP_BrowserReload))
        btn_ref.clicked.connect(self._refresh_ports)
        lay.addWidget(btn_ref)

        lay.addWidget(QLabel("Baud:"))
        self.cb_baud = QComboBox()
        for b in ["9600","19200","38400","57600","115200","230400","460800","921600"]:
            self.cb_baud.addItem(b)
        self.cb_baud.setCurrentText("115200")
        lay.addWidget(self.cb_baud)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.clicked.connect(self._connect)
        lay.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._disconnect)
        lay.addWidget(self.btn_disconnect)

        lay.addStretch()
        self.lbl_conn = QLabel("● OFFLINE")
        self.lbl_conn.setStyleSheet("color:#ff4d6d; font-weight:700;")
        lay.addWidget(self.lbl_conn)
        lay.addStretch()

        self.btn_record = QPushButton("⏺  Save CSV")
        self.btn_record.setObjectName("btn_record")
        self.btn_record.clicked.connect(self._toggle_record)
        lay.addWidget(self.btn_record)

        self._refresh_ports()
        return bar

    # ── TABS ────────────────────────────────────────────────
    def _tab_dashboard(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(16, 16, 16, 16); v.setSpacing(12)

        grid = QWidget(); g = QGridLayout(grid); g.setSpacing(10)
        self.card_pvdd   = MetricCard("PVDD",        "V",   warn=16, danger=18)
        self.card_i1     = MetricCard("Current 1",   "A",   warn=20, danger=35)
        self.card_i2     = MetricCard("Current 2",   "A",   warn=20, danger=35)
        self.card_i3     = MetricCard("Current 3",   "A",   warn=20, danger=35)
        self.card_rpm    = MetricCard("Motor RPM",   "rpm")
        self.card_redrpm = MetricCard("Reducer RPM", "rpm")
        self.card_ntc    = MetricCard("Temperature", "°C",  warn=60, danger=80)
        self.card_state  = MetricCard("State")
        for i, c in enumerate([self.card_pvdd, self.card_i1, self.card_i2, self.card_i3,
                                self.card_rpm, self.card_redrpm, self.card_ntc, self.card_state]):
            g.addWidget(c, i // 4, i % 4)
        v.addWidget(grid)

        dig = QGroupBox("Digital Status"); dg = QHBoxLayout(dig)
        self.lbl_id     = QLabel("ID: —")
        self.lbl_nfault = QLabel("NFault: —")
        self.lbl_noctw  = QLabel("OC Warn: —")
        #self.lbl_ffall   = QLabel("Free Fall: —")
        self.lbl_ang    = QLabel("Motor Angle: —°")
        self.lbl_redang = QLabel("Reducer Angle: —°")
        for l in [self.lbl_id, self.lbl_nfault, self.lbl_noctw, self.lbl_ang, self.lbl_redang]:
            l.setStyleSheet("font-family:'Consoles'; font-size:13px; padding:4px 12px;")
            dg.addWidget(l)
        dg.addStretch(); v.addWidget(dig)

        pw, self.crv_dash_rpm = make_plot("Motor RPM (quick view)", "rpm", ['#00d4aa','#4e9fff'])
        pw.addLegend()
        self.crv_dash_rpm[0].setData(name="Motor RPM")
        self.crv_dash_rpm[1].setData(name="Reducer RPM")
        v.addWidget(pw)
        return w

    def _tab_currents(self):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
        pw1, self.crv_cur = make_plot("Phase Currents", "Amps", ['#ff4d6d','#ffb347','#4e9fff'])
        pw1.addLegend()
        self.crv_cur[0].setData(name="I1"); self.crv_cur[1].setData(name="I2"); self.crv_cur[2].setData(name="I3")
        pw2, self.crv_raw = make_plot("ADC phases (A, B, C)", "Volts", ['#ff4d6d','#4e9fff','#00d4aa'])
        pw2.addLegend()
        self.crv_raw[0].setData(name="A"); self.crv_raw[1].setData(name="B"); self.crv_raw[2].setData(name="C")
        v.addWidget(pw1); v.addWidget(pw2)
        return w

    def _tab_mechanical(self):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
        pw1, self.crv_rpm = make_plot("Angular velocity", "RPM", ['#00d4aa','#4e9fff'])
        pw1.addLegend()
        self.crv_rpm[0].setData(name="Motor"); self.crv_rpm[1].setData(name="Reducer")
        pw2, self.crv_ang = make_plot("Angle", "degrees", ['#c77dff'])
        pw2.addLegend(); self.crv_ang[0].setData(name="Motor angle")
        v.addWidget(pw1); v.addWidget(pw2)
        return w
    
    # def _tab_gyro(self):
    #     # Commented out to disable gyro/accel tab
    #     w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
    #     pw1, self.crv_acc = make_plot("Acceleration", "G", ['#ff4d6d','#ffb347','#4e9fff'])
    #     pw1.addLegend()
    #     self.crv_acc[0].setData(name="Acc X"); self.crv_acc[1].setData(name="Acc Y"); self.crv_acc[2].setData(name="Acc Z")
    #     pw2, self.crv_gyr = make_plot("Gyroscope", "dps", ['#ff4d6d','#ffb347','#4e9fff'])
    #     pw2.addLegend()
    #     self.crv_gyr[0].setData(name="Gyr X"); self.crv_gyr[1].setData(name="Gyr Y"); self.crv_gyr[2].setData(name="Gyr Z")
    #     v.addWidget(pw1); v.addWidget(pw2)
    #     return w


    def _tab_power(self):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
        pw1, self.crv_pvdd = make_plot("Power Supply", "Volts", ['#ffb347'])
        pw1.addLegend(); self.crv_pvdd[0].setData(name="PVDD")
        pw1.addItem(pg.InfiniteLine(pos=16, angle=0, pen=pg.mkPen('#ff4d6d', width=1, style=Qt.PenStyle.DashLine)))
        pw2, self.crv_ntc = make_plot("Temperature NTC", "°C", ['#ff6b6b'])
        pw2.addLegend(); self.crv_ntc[0].setData(name="NTC")
        pw2.addItem(pg.InfiniteLine(pos=25, angle=0, pen=pg.mkPen('#ffb347', width=1, style=Qt.PenStyle.DashLine)))
        v.addWidget(pw1); v.addWidget(pw2)
        return w

    def _tab_console(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(16, 16, 16, 16); v.setSpacing(8)
        h = QHBoxLayout()
        self.chk_raw   = QCheckBox("Show all lines"); self.chk_raw.setChecked(True)
        self.chk_pause = QCheckBox("Pause scroll")
        btn_clear = QPushButton("Clear"); btn_clear.clicked.connect(lambda: self.console.clear())
        h.addWidget(self.chk_raw); h.addWidget(self.chk_pause); h.addStretch(); h.addWidget(btn_clear)
        v.addLayout(h)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        v.addWidget(self.console)
        return w

    def _tab_control(self):
        from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QGridLayout, QLabel, QComboBox, QPushButton, QLineEdit, QHBoxLayout
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(24, 24, 24, 24); v.setSpacing(16)

        # --- Motor State Group ---
        grp_state = QGroupBox("Motor state"); gs = QGridLayout(grp_state)
        self.lbl_ctrl_state  = QLabel("—"); self.lbl_ctrl_rpm   = QLabel("—")
        self.lbl_ctrl_pvdd   = QLabel("—"); self.lbl_ctrl_nfault = QLabel("—")
        self.lbl_ctrl_id     = QLabel("—")
        
        state_items = [
            ("State:", self.lbl_ctrl_state), ("Motor RPM:", self.lbl_ctrl_rpm),
            ("PVDD:", self.lbl_ctrl_pvdd), ("NFAULT:", self.lbl_ctrl_nfault),
            ("Motor ID:", self.lbl_ctrl_id)
        ]
        
        for i, (name, lbl) in enumerate(state_items):
            gs.addWidget(QLabel(name), i // 2, (i % 2) * 2)
            gs.addWidget(lbl,          i // 2, (i % 2) * 2 + 1)
        v.addWidget(grp_state)

        # --- Commands Group ---
        grp_cmd = QGroupBox("Commands (sends via UART)"); gc = QGridLayout(grp_cmd); gc.setSpacing(10)
        
        # Define columns stretch to ensure 1/4 width for each of the 4 columns
        gc.setColumnStretch(0, 1) # Column for IDLE
        gc.setColumnStretch(1, 1) # Column for ALIGN
        gc.setColumnStretch(2, 1) # Column for RUN + TEXT
        gc.setColumnStretch(3, 1) # Column for BRAKE

        btn_idle  = QPushButton("⏸  IDLE")
        btn_align = QPushButton("⚙  Align")
        btn_stop  = QPushButton("⏹  Brake")
        
        # Setup Run Button + Input group (This group will occupy the 3rd 1/4)
        self.btn_run = QPushButton("▶  RUN")
        self.run_input = QLineEdit()
        self.run_input.setPlaceholderText("Freq")
        self.run_input.setText("30")
        self.run_input.setMinimumWidth(40) # Ensure it's not too squished

        run_layout = QHBoxLayout()
        run_layout.setSpacing(2)
        run_layout.setContentsMargins(0, 0, 0, 0) 
        run_layout.addWidget(self.btn_run, stretch=2) # Button gets more of the 1/4 space
        run_layout.addWidget(self.run_input, stretch=1) # Text gets less of the 1/4 space

        # Command Mapping
        cmd_map = [(btn_idle, "CMD:IDLE\n"), (btn_align, "CMD:ALIGN\n"),
                   (self.btn_run, "CMD:RUN"), (btn_stop, "CMD:BRAKE\n")]

        for btn, cmd in cmd_map:
            if cmd == "CMD:RUN":
                btn.clicked.connect(self._handle_run_command)
            else:
                btn.clicked.connect(lambda _, c=cmd: self._send_serial(c))

        # Dropdown and Stop logic
        self.cb_fbk = QComboBox()
        self.cb_fbk.addItems(["MAG ENCODER", "HALL SENSOR"])
        self.cb_fbk.currentTextChanged.connect(lambda m: self._send_serial(f"CMD:FBK:{m.upper().replace(' ','_')}\n"))

        self.cb_control_mode = QComboBox()
        self.cb_control_mode.addItems(["OPENLOOP_BEMF", "CLOSED_FOC", "CLOSED_SINUSOIDAL"])
        self.cb_control_mode.currentTextChanged.connect(lambda m: self._send_serial(f"CMD:CTRL:{m.upper().replace(' ','_')}\n"))

        self.btn_brake_ctrl = QPushButton("🛑  STOP")
        self.btn_brake_ctrl.setObjectName("btn_brake")
        self.btn_brake_ctrl.clicked.connect(lambda: self._send_serial("CMD:STOP\n"))

        # --- Organize Grid (gc) ---
        gc.addWidget(btn_idle, 0, 0)
        gc.addWidget(btn_align, 0, 1)
        gc.addLayout(run_layout, 0, 2) 
        gc.addWidget(btn_stop, 0, 3)

        # Feedback row spans all 4 columns for clean look
        gc.addWidget(QLabel("Feedback Method:"), 1, 0)
        gc.addWidget(self.cb_fbk, 1, 1, 1, 1) 

        gc.addWidget(QLabel("Control Mode:"), 1, 2)
        gc.addWidget(self.cb_control_mode, 1, 3, 1, 1)

        # Large Stop button spans all 4 columns
        gc.addWidget(self.btn_brake_ctrl, 2, 0, 1, 4)
        v.addWidget(grp_cmd)

        # --- Free Text Group ---
        grp_tx = QGroupBox("Send free text"); gt = QHBoxLayout(grp_tx)
        self.tx_edit = QLineEdit(); self.tx_edit.setPlaceholderText("Write command and press Enter…")
        self.tx_edit.returnPressed.connect(self._send_manual)
        btn_send = QPushButton("Send"); btn_send.clicked.connect(self._send_manual)
        gt.addWidget(self.tx_edit); gt.addWidget(btn_send)
        
        v.addWidget(grp_tx); 

            # --- Alpha Slider Group ---
        grp_alpha = QGroupBox("Change parameter"); ga = QVBoxLayout(grp_alpha); ga.setSpacing(6)

        # Top row: label name input + value display
        alpha_top = QHBoxLayout()
        alpha_lbl = QLabel("Parameter name:")
        alpha_lbl.setStyleSheet("color:#7a8399; font-size:11px;")
        self.alpha_name = QLineEdit()
        self.alpha_name.setText("alpha")
        self.alpha_name.setFixedWidth(100)
        self.alpha_name.setStyleSheet("background:#161b27; border:1px solid #2a3348; border-radius:4px; padding:3px 8px; color:#c8cdd8;")

        self.alpha_min_input = QLineEdit(); self.alpha_min_input.setText("0")
        self.alpha_min_input.setFixedWidth(55)
        self.alpha_min_input.setStyleSheet("background:#161b27; border:1px solid #2a3348; border-radius:4px; padding:3px 6px; color:#c8cdd8;")
        self.alpha_max_input = QLineEdit(); self.alpha_max_input.setText("100")
        self.alpha_max_input.setFixedWidth(55)
        self.alpha_max_input.setStyleSheet("background:#161b27; border:1px solid #2a3348; border-radius:4px; padding:3px 6px; color:#c8cdd8;")

        self.alpha_val_lbl = QLabel("0")
        self.alpha_val_lbl.setFixedWidth(50)
        self.alpha_val_lbl.setStyleSheet("color:#00d4aa; font-size:15px; font-weight:700; font-family:'Consoles';")
        self.alpha_val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        alpha_top.addWidget(alpha_lbl)
        alpha_top.addWidget(self.alpha_name)
        alpha_top.addSpacing(16)
        alpha_top.addWidget(QLabel("Min:")); alpha_top.addWidget(self.alpha_min_input)
        alpha_top.addSpacing(8)
        alpha_top.addWidget(QLabel("Max:")); alpha_top.addWidget(self.alpha_max_input)
        alpha_top.addStretch()
        alpha_top.addWidget(QLabel("Value:")); alpha_top.addWidget(self.alpha_val_lbl)
        ga.addLayout(alpha_top)

        # Slider — full width
        self.alpha_slider = QSlider(Qt.Orientation.Horizontal)
        self.alpha_slider.setMinimum(0)
        self.alpha_slider.setMaximum(1000)   # internal resolution 0–1000 for smooth float mapping
        self.alpha_slider.setValue(0)
        self.alpha_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px; background: #1f2535; border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #00d4aa; border: none;
                width: 16px; height: 16px; margin: -5px 0; border-radius: 8px;
            }
            QSlider::sub-page:horizontal { background: #00d4aa; border-radius: 3px; }
        """)
        ga.addWidget(self.alpha_slider)

        # Send button row
        alpha_btn_row = QHBoxLayout()
        self.alpha_send_btn = QPushButton("Send")
        self.alpha_send_btn.setFixedWidth(80)
        self.alpha_live_chk = QPushButton("Live  ○")
        self.alpha_live_chk.setCheckable(True)
        self.alpha_live_chk.setFixedWidth(80)
        self.alpha_live_chk.setStyleSheet("""
            QPushButton { background:#1a2030; color:#c8cdd8; border:1px solid #2a3348; border-radius:6px; padding:5px; }
            QPushButton:checked { background:#0d3d2e; color:#00d4aa; border-color:#00d4aa44; }
        """)
        self.alpha_live_chk.toggled.connect(lambda on: self.alpha_live_chk.setText("Live  ●" if on else "Live  ○"))
        alpha_btn_row.addStretch()
        alpha_btn_row.addWidget(self.alpha_live_chk)
        alpha_btn_row.addWidget(self.alpha_send_btn)
        ga.addLayout(alpha_btn_row)

        v.addWidget(grp_alpha)

        # --- Slider logic ---
        def _alpha_value():
            try:
                mn = float(self.alpha_min_input.text())
                mx = float(self.alpha_max_input.text())
            except ValueError:
                mn, mx = 0.0, 1.0
            return mn + (self.alpha_slider.value() / 1000.0) * (mx - mn)

        def _on_slider_moved():
            val = _alpha_value()
            # Format as int if whole number, else 2 decimal places
            self.alpha_val_lbl.setText(f"{val:.0f}" if val == int(val) else f"{val:.2f}")
            if self.alpha_live_chk.isChecked():
                _send_alpha()

        def _send_alpha():
            val = _alpha_value()
            name = self.alpha_name.text().strip() or "alpha"
            fmt = f"{val:.0f}" if val == int(val) else f"{val:.2f}"
            self._send_serial(f"CMD:{name.upper()}:{fmt}\n")

        self.alpha_slider.valueChanged.connect(_on_slider_moved)
        self.alpha_send_btn.clicked.connect(_send_alpha)
        # Update display when min/max change
        self.alpha_min_input.textChanged.connect(lambda _: _on_slider_moved())
        self.alpha_max_input.textChanged.connect(lambda _: _on_slider_moved())

        v.addStretch()
        return w


    # ── SERIAL ───────────────────────────────────────────────
    def _refresh_ports(self):
        self.cb_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports, key=lambda x: x.device):
            self.cb_port.addItem(f"{p.device}  —  {p.description[:40]}", p.device)
        if not ports:
            self.cb_port.addItem("(No ports detected)", "")
    
    def _handle_run_command(self):
        val = self.run_input.text().strip()
        if val:
            # Sends as "CMD:RUN:30\n"
            self._send_serial(f"CMD:RUN:{val}\n")
        else:
            # Fallback to standard RUN if empty
            self._send_serial("CMD:RUN\n")

    def _connect(self):
        port = self.cb_port.currentData()
        if not port:
            self._status("Select a valid port", "err"); return
        baud = int(self.cb_baud.currentText())
        self.worker = SerialWorker(port, baud)
        self.worker.data_ready.connect(self._on_data)
        self.worker.raw_line.connect(self._on_raw)
        self.worker.error.connect(self._on_recoverable_error)
        self.worker.fatal.connect(self._on_fatal_error)
        self.worker.start()
        self._plot_timer.start()
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.lbl_conn.setText("● ONLINE")
        self.lbl_conn.setStyleSheet("color:#00d4aa; font-weight:700;")
        self._status(f"Connected to {port} @ {baud} baud", "ok")

    def _disconnect(self):
        if self.worker:
            self.worker.stop(); self.worker = None
        self._plot_timer.stop()
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.lbl_conn.setText("● OFFLINE")
        self.lbl_conn.setStyleSheet("color:#ff4d6d; font-weight:700;")
        self._status("Disconnected", "warn")
        if self.recording: self._toggle_record()

    def _send_serial(self, cmd):
        if self.worker and self.worker.isRunning():
            self.worker.write_data(cmd)
            self._status(f"TX → {cmd.strip()}", "ok")
        else:
            self._status("No active connection", "warn")

    def _send_manual(self):
        txt = self.tx_edit.text().strip()
        if txt: self._send_serial(txt + "\n");self.tx_edit.clear()

    # ── DATA ─────────────────────────────────────────────────
    def _on_data(self, d):
        for k in self.buf:
            if k in d: self.buf[k].append(d[k])

        self.card_pvdd.update(d['pvdd'],      ".2f")
        self.card_i1.update(abs(d['i1']),     ".2f")
        self.card_i2.update(abs(d['i2']),     ".2f")
        self.card_i3.update(abs(d['i3']),     ".2f")
        self.card_rpm.update(d['m_rpm'],      ".1f")
        self.card_redrpm.update(d['red_rpm'], ".2f")
        self.card_ntc.update(d['ntc'],        ".1f")
        self.card_state.lbl_val.setText(state_str)

        nf_ok = d['nfault'] == 1; oc_ok = d['noctw'] == 1
        self.lbl_id.setText(f"ID: {d['ID']}")
        self.lbl_nfault.setText(f"NFault: {'OK' if nf_ok else 'FAULT'}")
        self.lbl_nfault.setStyleSheet(f"color:{'#00d4aa' if nf_ok else '#ff4d6d'}; font-family:'Consoles'; padding:4px 12px;")
        self.lbl_noctw.setText(f"OC Warn: {'OK' if oc_ok else 'WARN'}")
        self.lbl_noctw.setStyleSheet(f"color:{'#00d4aa' if oc_ok else '#ffb347'}; font-family:'Consoles'; padding:4px 12px;")
        #self.lbl_ffall.setText(f"Free Fall: {'YES' if d['ffall'] == 1 else 'NO'}")
        #self.lbl_ffall.setStyleSheet(f"color:{'#ff4d6d' if d['ffall'] == 1 else '#00d4aa'}; font-family:'Consoles'; padding:4px 12px;")
        self.lbl_ang.setText(f"Motor angle: {d['m_ang']:.2f}°")
        self.lbl_redang.setText(f"Reducer angle: {d['red_ang']:.2f}°")

        self.lbl_ctrl_state.setText(state_str)
        self.lbl_ctrl_rpm.setText(f"{d['m_rpm']:.1f} rpm")
        self.lbl_ctrl_pvdd.setText(f"{d['pvdd']:.2f} V")
        self.lbl_ctrl_nfault.setText("OK" if nf_ok else "FAULT")
        self.lbl_ctrl_id.setText(str(d['ID']))

        if self.recording and self.rec_writer:
            self.rec_writer.writerow([datetime.utcnow().isoformat(),
                d['pvdd'], d['i1'], d['i2'], d['i3'], d['a'], d['b'], d['c'],
                d['nfault'], d['noctw'], d['ntc'], d['m_ang'], d['red_ang'],
                d['m_rpm'], d['red_rpm']])

    def _on_raw(self, line):
        if self.chk_raw.isChecked():
            self.console.append(line)
            if not self.chk_pause.isChecked():
                self.console.verticalScrollBar().setValue(
                    self.console.verticalScrollBar().maximum())

    def _on_recoverable_error(self, msg):
        self._status(f"⚠ {msg}", "warn")
        self.console.append(f"[WARN] {msg}")

    def _on_fatal_error(self, msg):
        self._status(f"✖ {msg}", "err")
        self.console.append(f"[FATAL] {msg}")
        self._disconnect()

    # ── PLOTS ────────────────────────────────────────────────
    def _refresh_plots(self):
        arr = {k: np.array(self.buf[k]) for k in self.buf}
        self.crv_dash_rpm[0].setData(self.x, arr['m_rpm'])
        self.crv_dash_rpm[1].setData(self.x, arr['red_rpm'])
        self.crv_cur[0].setData(self.x, arr['i1'])
        self.crv_cur[1].setData(self.x, arr['i2'])
        self.crv_cur[2].setData(self.x, arr['i3'])
        self.crv_raw[0].setData(self.x, arr['a'])
        self.crv_raw[1].setData(self.x, arr['b'])
        self.crv_raw[2].setData(self.x, arr['c'])
        self.crv_rpm[0].setData(self.x, arr['m_rpm'])
        self.crv_rpm[1].setData(self.x, arr['red_rpm'])
        self.crv_ang[0].setData(self.x, arr['m_ang'])
        self.crv_pvdd[0].setData(self.x, arr['pvdd'])
        self.crv_ntc[0].setData(self.x, arr['ntc'])


    # ── CSV ──────────────────────────────────────────────────
    def _toggle_record(self):
        if not self.recording:
            path, _ = QFileDialog.getSaveFileName(self, "Save CSV",
                f"telem_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "CSV (*.csv)")
            if not path: return
            self.rec_file   = open(path, 'w', newline='')
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow(['timestamp','pvdd','i1','i2','i3',
                'a','b','c','nfault','noctw','ntc','m_ang','red_ang','m_rpm','red_rpm','accx','accy','accz','gyrx','gyry','gyrz','state'])
            self.recording = True
            self.btn_record.setText("⏹  Stop Recording")
            self._status(f"Recording → {path}", "ok")
        else:
            self.recording = False
            if self.rec_file: self.rec_file.close(); self.rec_file = self.rec_writer = None
            self.btn_record.setText("⏺  Save CSV")
            self._status("Recording stopped", "warn")

    def _status(self, msg, level="ok"):
        colors = {"ok":"#00d4aa", "warn":"#ffb347", "err":"#ff4d6d"}
        self.status.showMessage(f"  {msg}")
        self.status.setStyleSheet(f"background:#080b10; color:{colors.get(level,'#c8cdd8')};"
                                   "border-top:1px solid #1a2030; font-family:'Consoles'; font-size:12px;")

    def closeEvent(self, e):
        if self.worker: self.worker.stop()
        if self.rec_file: self.rec_file.close()
        e.accept()

# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("ASTURION GUI Monitor")
    app.setWindowIcon(QIcon("Flag_of_Asturias.svg.png"))
    win = MotorMonitor()
    win.show()
    sys.exit(app.exec())