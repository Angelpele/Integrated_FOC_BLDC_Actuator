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
    QStatusBar, QFrame, QCheckBox, QSlider, QMessageBox, QStyle,
    QLineEdit, QScrollArea, QDoubleSpinBox, QSizePolicy
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
QPushButton#btn_align { background: #1a2d1a; color: #7aff99; border-color: #7aff9944; }
QPushButton#btn_run   { background: #0d2e3d; color: #4e9fff; border-color: #4e9fff44; }
QComboBox, QSpinBox, QDoubleSpinBox {
    background: #161b27; border: 1px solid #2a3348;
    border-radius: 5px; padding: 5px 10px; color: #c8cdd8;
}
QComboBox::drop-down { border: none; width: 22px; }
QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { border-color: #3a4560; }
QGroupBox {
    border: 1px solid #1f2535; border-radius: 8px;
    margin-top: 12px; padding-top: 8px; font-weight: 600; color: #7a8399;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
QTextEdit, QLineEdit {
    background: #080b10; border: 1px solid #1f2535; border-radius: 6px;
    color: #c8cdd8; padding: 6px;
}
QTextEdit { color: #7aff99; font-family: 'Consolas', monospace; font-size: 12px; }
QLabel#metric_val  { font-size: 22px; font-weight: 700; color: #c8cdd8; }
QLabel#metric_unit { font-size: 11px; color: #5a6070; }
QLabel#metric_name { font-size: 11px; color: #7a8399; }
QStatusBar { background: #080b10; border-top: 1px solid #1f2535; }
QCheckBox::indicator {
    width: 16px; height: 16px; border: 1px solid #2a3348;
    border-radius: 3px; background: #161b27;
}
QCheckBox::indicator:checked { background: #00d4aa; border-color: #00d4aa; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical {
    background: #0e1117; width: 8px; border-radius: 4px;
}
QScrollBar::handle:vertical { background: #2a3348; border-radius: 4px; min-height: 20px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""

SLIDER_STYLE = """
QSlider::groove:horizontal {
    height: 6px; background: #1f2535; border-radius: 3px;
}
QSlider::handle:horizontal {
    background: {color}; border: none;
    width: 14px; height: 14px; margin: -4px 0; border-radius: 7px;
}
QSlider::sub-page:horizontal {{ background: {color}; border-radius: 3px; }}
"""

PLOT_LEN   = 600
state_str  = "—"

# ─────────────────────────────────────────────────────────────
#  SERIAL WORKER
# ─────────────────────────────────────────────────────────────
class SerialWorker(QThread):
    data_ready = pyqtSignal(dict)
    raw_line   = pyqtSignal(str)
    error      = pyqtSignal(str)
    fatal      = pyqtSignal(str)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = True
        self.ser = None

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
                                'iq':      float(parts[2]),
                                'id':      float(parts[3]),
                                'i3':      float(parts[4]),
                                'a':       float(parts[5]),
                                'b':       float(parts[6]),
                                'c':       float(parts[7]),
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
#  PID ROW WIDGET
# ─────────────────────────────────────────────────────────────
class PIDRow(QWidget):
    """One row: label + Kp, Ki, Kd sliders + numeric display + send button."""

    def __init__(self, loop_name, prefix, color, send_callback, parent=None):
        super().__init__(parent)
        self.prefix = prefix
        self.send_cb = send_callback
        self.color = color

        main = QVBoxLayout(self)
        main.setContentsMargins(0, 4, 0, 4)
        main.setSpacing(4)

        # Title
        title = QLabel(f"  {loop_name}")
        title.setStyleSheet(f"color:{color}; font-weight:700; font-size:13px; background:#161b27;"
                             "border-radius:4px; padding:3px 8px;")
        main.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setContentsMargins(4, 0, 4, 0)

        self.params = {}   # name -> {slider, val_lbl, spinbox}
        params_cfg = [
            ("Kp", 0.0, 10.0,  0.001,  "#00d4aa"),
            ("Ki", 0.0,  100.0,  0.01, "#4e9fff"),
            ("Kd", 0.0,  20.0,  0.01, "#ffb347"),
        ]
        for row_i, (name, mn, mx, step, pcol) in enumerate(params_cfg):
            lbl = QLabel(f"<b>{name}</b>")
            lbl.setFixedWidth(28)
            lbl.setStyleSheet("color:#9aa0b0; font-size:12px;")

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            slider.setValue(0)
            slider.setStyleSheet(SLIDER_STYLE.replace('{color}', pcol).replace('{{', '{').replace('}}', '}'))

            val_lbl = QLabel("0.000")
            val_lbl.setFixedWidth(52)
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            val_lbl.setStyleSheet(f"color:{pcol}; font-family:'Consolas'; font-size:12px; font-weight:700;")

            # Store config for conversion
            slider._mn = mn
            slider._mx = mx
            slider._name = name
            slider._val_lbl = val_lbl

            def _on_change(v, s=slider, vl=val_lbl, mn=mn, mx=mx):
                fval = mn + (v / 1000.0) * (mx - mn)
                vl.setText(f"{fval:.3f}")

            slider.valueChanged.connect(_on_change)

            grid.addWidget(lbl,     row_i, 0)
            grid.addWidget(slider,  row_i, 1)
            grid.addWidget(val_lbl, row_i, 2)

            self.params[name] = {'slider': slider, 'val_lbl': val_lbl, 'mn': mn, 'mx': mx}

        # Send button
        btn = QPushButton("Send")
        btn.setFixedWidth(64)
        btn.setStyleSheet(f"background:#1a2030; color:{color}; border:1px solid {color}55;"
                           "border-radius:5px; padding:4px 8px; font-weight:600;")
        btn.clicked.connect(self._send)
        grid.addWidget(btn, 0, 3, 3, 1, Qt.AlignmentFlag.AlignVCenter)

        grid.setColumnStretch(1, 1)
        main.addLayout(grid)

    def _get_val(self, name):
        p = self.params[name]
        return p['mn'] + (p['slider'].value() / 1000.0) * (p['mx'] - p['mn'])

    def _send(self):
        kp = self._get_val("Kp")
        ki = self._get_val("Ki")
        kd = self._get_val("Kd")
        self.send_cb(f"CMD:PID_{self.prefix}_KP:{kp:.4f}_KI:{ki:.4f}_KD:{kd:.4f}\n")

    def get_values(self):
        return {n: self._get_val(n) for n in self.params}


# ─────────────────────────────────────────────────────────────
#  PLOT HELPER
# ─────────────────────────────────────────────────────────────
def make_plot(title, ylabel, colors, legend=True):
    pw = pg.PlotWidget()
    pw.setTitle(title, color='#7a8399', size='11pt')
    pw.setLabel('left', ylabel, color='#5a6070')
    pw.showGrid(x=True, y=True, alpha=0.15)
    pw.getAxis('bottom').setStyle(tickFont=QFont('Consolas', 9))
    pw.getAxis('left').setStyle(tickFont=QFont('Consolas', 9))
    pw.setBackground('#0e1117')
    if legend:
        pw.addLegend(offset=(10, 5))
    return pw, [pw.plot(pen=pg.mkPen(color=c, width=1.8)) for c in colors]


# ─────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────
class MotorMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Monitor  //  ESP32-S3 Controller")
        self.resize(1340, 880)
        self.worker = None
        self.recording = False
        self.rec_file = self.rec_writer = None

        N = PLOT_LEN
        self.buf = {k: collections.deque([0.0]*N, maxlen=N) for k in
                    ['pvdd','iq','id','i3','m_rpm','red_rpm','ntc','m_ang',
                     'red_ang','a','b','c',
                     'sp_pos','sp_vel','sp_torque']}
        self.x = np.arange(N)

        # Setpoints (user-defined targets)
        self.sp_pos    = 0.0
        self.sp_vel    = 0.0
        self.sp_torque = 0.0

        self._build_ui()
        self.setStyleSheet(DARK_STYLE)

        self._plot_timer = QTimer()
        self._plot_timer.setInterval(33)   # ~30 fps
        self._plot_timer.timeout.connect(self._refresh_plots)

    # ─── UI CONSTRUCTION ──────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_toolbar())

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)

        # ── ORDER: Control first ──────────────────────────────
        self.tabs.addTab(self._tab_control(),    "  Control  ")
        self.tabs.addTab(self._tab_dashboard(),  "  Dashboard  ")
        self.tabs.addTab(self._tab_currents(),   "  Currents  ")
        self.tabs.addTab(self._tab_mechanical(), "  Mechanical  ")
        self.tabs.addTab(self._tab_power(),      "  Power · NTC  ")
        self.tabs.addTab(self._tab_console(),    "  Raw Console  ")

        root.addWidget(self.tabs)

        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self._status("No Connection", "warn")

    # ─── TOOLBAR ─────────────────────────────────────────────
    def _build_toolbar(self):
        bar = QWidget()
        bar.setStyleSheet("background:#080b10; border-bottom:1px solid #1a2030;")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 8, 16, 8)
        lay.setSpacing(10)

        logo = QLabel("ASTURION")
        logo.setStyleSheet("font-family:'Consolas'; font-size:14px; font-weight:700;"
                           "color:#00d4aa; letter-spacing:3px;")
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

    # ─── TAB: CONTROL ─────────────────────────────────
    def _tab_control(self):
        # Top-level: left panel (commands + PID) + right panel (plots)
        outer = QWidget()
        h_root = QHBoxLayout(outer)
        h_root.setContentsMargins(0, 0, 0, 0)
        h_root.setSpacing(0)

        # ── LEFT PANEL (scrollable) ──────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(460)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        left_content = QWidget()
        left = QVBoxLayout(left_content)
        left.setContentsMargins(16, 16, 12, 16)
        left.setSpacing(10)

        # ── Motor Commands ───────────────────────────────────
        grp_cmd = QGroupBox("Motor Commands")
        gc = QGridLayout(grp_cmd)
        gc.setSpacing(8)
        gc.setContentsMargins(10, 16, 10, 10)

        self.btn_align = QPushButton("⚙  ALIGN")
        self.btn_align.setObjectName("btn_align")
        self.btn_run   = QPushButton("▶  RUN")
        self.btn_run.setObjectName("btn_run")
        self.btn_idle  = QPushButton("⏸  IDLE")
        btn_brake      = QPushButton("🛑  BRAKE")
        btn_brake.setObjectName("btn_brake")

        self.run_dur = QLineEdit(); self.run_dur.setPlaceholderText("Duration (s)")
        self.run_dur.setText("10"); self.run_dur.setFixedWidth(90)

        self.btn_align.clicked.connect(lambda: self._send_serial("CMD:ALIGN\n"))
        self.btn_run.clicked.connect(self._handle_run)
        self.btn_idle.clicked.connect(lambda: self._send_serial("CMD:IDLE\n"))
        btn_brake.clicked.connect(lambda: self._send_serial("CMD:BRAKE\n"))

        gc.addWidget(self.btn_align, 0, 0)
        gc.addWidget(self.btn_run,   0, 1)
        gc.addWidget(self.run_dur,   0, 2)
        gc.addWidget(self.btn_idle,  1, 0)
        gc.addWidget(btn_brake,      1, 1, 1, 2)

        # Feedback + Control mode dropdowns
        row_fb = QHBoxLayout()
        row_fb.addWidget(QLabel("Feedback:"))
        self.cb_fbk = QComboBox()
        self.cb_fbk.addItems(["MAG_ENCODER", "HALL_SENSOR"])
        self.cb_fbk.currentTextChanged.connect(
            lambda m: self._send_serial(f"CMD:FBK:{m}\n"))
        row_fb.addWidget(self.cb_fbk)

        row_ctrl = QHBoxLayout()
        row_ctrl.addWidget(QLabel("Mode:"))
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(["OPENLOOP_BEMF", "CLOSED_FOC", "CLOSED_SINUSOIDAL"])
        self.cb_mode.currentTextChanged.connect(
            lambda m: self._send_serial(f"CMD:CTRL:{m}\n"))
        row_ctrl.addWidget(self.cb_mode)

        gc.addLayout(row_fb,   2, 0, 1, 3)
        gc.addLayout(row_ctrl, 3, 0, 1, 3)
        left.addWidget(grp_cmd)

        # ── Setpoints ────────────────────────────────────────
        grp_sp = QGroupBox("Setpoints")
        sp_lay = QGridLayout(grp_sp)
        sp_lay.setSpacing(8)
        sp_lay.setContentsMargins(10, 16, 10, 10)

        def _sp_row(label, unit, default, row):
            lbl = QLabel(label)
            lbl.setFixedWidth(100)
            sb = QDoubleSpinBox()
            sb.setDecimals(2)
            sb.setRange(-9999, 9999)
            sb.setValue(default)
            sb.setSuffix(f"  {unit}")
            btn = QPushButton("Set")
            btn.setFixedWidth(55)
            sp_lay.addWidget(lbl, row, 0)
            sp_lay.addWidget(sb,  row, 1)
            sp_lay.addWidget(btn, row, 2)
            return sb, btn

        self.sp_pos_sb,    btn_sp_pos    = _sp_row("Position",  "°",   0.0, 0)
        self.sp_vel_sb,    btn_sp_vel    = _sp_row("Velocity",  "rpm", 0.0, 1)
        self.sp_torque_sb, btn_sp_torque = _sp_row("Torque",    "A",   0.0, 2)

        btn_sp_pos.clicked.connect(lambda: self._set_sp('pos'))
        btn_sp_vel.clicked.connect(lambda: self._set_sp('vel'))
        btn_sp_torque.clicked.connect(lambda: self._set_sp('torque'))
        left.addWidget(grp_sp)

        # ── Custom command ───────────────────────────────────
        grp_tx = QGroupBox("Custom UART Command")
        gt = QHBoxLayout(grp_tx)
        gt.setContentsMargins(10, 16, 10, 10)
        self.tx_edit = QLineEdit()
        self.tx_edit.setPlaceholderText("e.g.  CMD:BETA:0.85")
        self.tx_edit.returnPressed.connect(self._send_manual)
        btn_send = QPushButton("Send")
        btn_send.clicked.connect(self._send_manual)
        gt.addWidget(self.tx_edit)
        gt.addWidget(btn_send)
        left.addWidget(grp_tx)

        # ── PID Tuning ───────────────────────────────────────
        grp_pid = QGroupBox("PID Tuning")
        pid_lay = QVBoxLayout(grp_pid)
        pid_lay.setContentsMargins(10, 16, 10, 10)
        pid_lay.setSpacing(10)

        note = QLabel("  Sends CMD:PID_<LOOP>_K<X>:<val> on Send")
        note.setStyleSheet("color:#5a6070; font-size:11px; font-style:italic;")
        pid_lay.addWidget(note)

        sep_style = "background:#1f2535; max-height:1px; margin:2px 0;"

        self.pid_pos    = PIDRow("Position Loop",  "POS",    "#c77dff", self._send_serial)
        self.pid_vel    = PIDRow("Velocity Loop",  "VEL",    "#00d4aa", self._send_serial)
        self.pid_torque = PIDRow("Torque (Iq) Loop","TORQUE","#ffb347", self._send_serial)
        self.pid_flux     = PIDRow("Flux (Id) Loop","FLUX",    "#4e9fff", self._send_serial)

        for i, pid in enumerate([self.pid_pos, self.pid_vel, self.pid_torque, self.pid_flux]):
            pid_lay.addWidget(pid)
            if i < 3:
                sep = QFrame(); sep.setStyleSheet(sep_style)
                pid_lay.addWidget(sep)

        left.addWidget(grp_pid)
        left.addStretch()

        scroll.setWidget(left_content)
        h_root.addWidget(scroll)

        # ── RIGHT PANEL (plots) ──────────────────────────────
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(8, 16, 16, 16)
        rv.setSpacing(8)

        # Position plot
        pw_pos, crv = make_plot("Reducer Position", "degrees",
                                ['#c77dff', '#c77dff55'])
        crv[0].setData(name="Measured")
        crv[1].setData(name="Setpoint")
        crv[1].setPen(pg.mkPen('#c77dff', width=1.5, style=Qt.PenStyle.DashLine))
        self.crv_ctrl_pos = crv

        # Velocity plot
        pw_vel, crv2 = make_plot("Velocity", "RPM",
                                 ['#00d4aa', '#00d4aa55'])
        crv2[0].setData(name="Measured")
        crv2[1].setData(name="Setpoint")
        crv2[1].setPen(pg.mkPen('#00d4aa', width=1.5, style=Qt.PenStyle.DashLine))
        self.crv_ctrl_vel = crv2

        # Torque / current plot
        pw_tor, crv3 = make_plot("Torque (Phase Current)", "A",
                                 ['#ffb347', '#ffb34755', '#4e9fff'])
        crv3[0].setData(name="Measured  iq")
        crv3[1].setData(name="Setpoint")
        crv3[1].setPen(pg.mkPen('#ffb347', width=1.5, style=Qt.PenStyle.DashLine))
        self.crv_ctrl_tor = crv3

        for pw in [pw_pos, pw_vel, pw_tor]:
            rv.addWidget(pw)

        h_root.addWidget(right, stretch=1)
        return outer

    # ─── TAB: DASHBOARD ───────────────────────────────────────
    def _tab_dashboard(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(16, 16, 16, 16); v.setSpacing(12)

        grid = QWidget(); g = QGridLayout(grid); g.setSpacing(10)
        self.card_pvdd   = MetricCard("PVDD",        "V",   warn=16, danger=18)
        self.card_iq     = MetricCard("Current Q",   "A",   warn=20, danger=35)
        self.card_id     = MetricCard("Current D",   "A",   warn=20, danger=35)
        self.card_i3     = MetricCard("Current 3",   "A",   warn=20, danger=35)
        self.card_rpm    = MetricCard("Motor RPM",   "rpm")
        self.card_redrpm = MetricCard("Reducer RPM", "rpm")
        self.card_ntc    = MetricCard("Temperature", "°C",  warn=60, danger=80)
        self.card_state  = MetricCard("State")
        for i, c in enumerate([self.card_pvdd, self.card_iq, self.card_id, self.card_i3,
                                self.card_rpm, self.card_redrpm, self.card_ntc, self.card_state]):
            g.addWidget(c, i // 4, i % 4)
        v.addWidget(grid)

        dig = QGroupBox("Digital Status"); dg = QHBoxLayout(dig)
        self.lbl_id     = QLabel("ID: —")
        self.lbl_nfault = QLabel("NFault: —")
        self.lbl_noctw  = QLabel("OC Warn: —")
        self.lbl_ang    = QLabel("Motor Angle: —°")
        self.lbl_redang = QLabel("Reducer Angle: —°")
        for l in [self.lbl_id, self.lbl_nfault, self.lbl_noctw,
                  self.lbl_ang, self.lbl_redang]:
            l.setStyleSheet("font-family:'Consolas'; font-size:13px; padding:4px 12px;")
            dg.addWidget(l)
        dg.addStretch(); v.addWidget(dig)

        pw, self.crv_dash_rpm = make_plot("Motor RPM (quick view)", "rpm",
                                          ['#00d4aa', '#4e9fff'])
        self.crv_dash_rpm[0].setData(name="Motor RPM")
        self.crv_dash_rpm[1].setData(name="Reducer RPM")
        v.addWidget(pw)
        return w

    # ─── TAB: CURRENTS ────────────────────────────────────────
    def _tab_currents(self):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
        pw1, self.crv_cur = make_plot("Phase Currents", "Amps",
                                      ['#ff4d6d','#ffb347','#4e9fff'])
        self.crv_cur[0].setData(name="iq")
        self.crv_cur[1].setData(name="id")
        self.crv_cur[2].setData(name="I3")
        pw2, self.crv_raw = make_plot("ADC Phases (A, B, C)", "raw",
                                      ['#ff4d6d','#4e9fff','#00d4aa'])
        self.crv_raw[0].setData(name="A")
        self.crv_raw[1].setData(name="B")
        self.crv_raw[2].setData(name="C")
        v.addWidget(pw1); v.addWidget(pw2)
        return w

    # ─── TAB: MECHANICAL ──────────────────────────────────────
    def _tab_mechanical(self):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
        pw1, self.crv_rpm = make_plot("Angular Velocity", "RPM",
                                      ['#00d4aa','#4e9fff'])
        self.crv_rpm[0].setData(name="Motor")
        self.crv_rpm[1].setData(name="Reducer")
        pw2, self.crv_ang = make_plot("Angle", "degrees", ['#c77dff','#ff7f50'])
        self.crv_ang[0].setData(name="Motor angle")
        self.crv_ang[1].setData(name="Reducer angle")
        v.addWidget(pw1); v.addWidget(pw2)
        return w

    # ─── TAB: POWER ───────────────────────────────────────────
    def _tab_power(self):
        w = QWidget(); v = QVBoxLayout(w); v.setContentsMargins(16, 16, 16, 16)
        pw1, self.crv_pvdd = make_plot("Power Supply", "Volts", ['#ffb347'])
        self.crv_pvdd[0].setData(name="PVDD")
        pw1.addItem(pg.InfiniteLine(
            pos=16, angle=0,
            pen=pg.mkPen('#ff4d6d', width=1, style=Qt.PenStyle.DashLine)))
        pw2, self.crv_ntc = make_plot("Temperature NTC", "°C", ['#ff6b6b'])
        self.crv_ntc[0].setData(name="NTC")
        pw2.addItem(pg.InfiniteLine(
            pos=60, angle=0,
            pen=pg.mkPen('#ffb347', width=1, style=Qt.PenStyle.DashLine)))
        v.addWidget(pw1); v.addWidget(pw2)
        return w

    # ─── TAB: CONSOLE ─────────────────────────────────────────
    def _tab_console(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(16, 16, 16, 16); v.setSpacing(8)
        h = QHBoxLayout()
        self.chk_raw   = QCheckBox("Show all lines"); self.chk_raw.setChecked(True)
        self.chk_pause = QCheckBox("Pause scroll")
        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(lambda: self.console.clear())
        h.addWidget(self.chk_raw)
        h.addWidget(self.chk_pause)
        h.addStretch()
        h.addWidget(btn_clear)
        v.addLayout(h)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        v.addWidget(self.console)
        return w

    # ─── SERIAL ───────────────────────────────────────────────
    def _refresh_ports(self):
        self.cb_port.clear()
        ports = serial.tools.list_ports.comports()
        for p in sorted(ports, key=lambda x: x.device):
            self.cb_port.addItem(f"{p.device}  —  {p.description[:40]}", p.device)
        if not ports:
            self.cb_port.addItem("(No ports detected)", "")

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
        if self.recording:
            self._toggle_record()

    def _send_serial(self, cmd):
        if self.worker and self.worker.isRunning():
            self.worker.write_data(cmd)
            self._status(f"TX → {cmd.strip()}", "ok")
        else:
            self._status("No active connection", "warn")

    def _send_manual(self):
        txt = self.tx_edit.text().strip()
        if txt:
            self._send_serial(txt + "\n")
            self.tx_edit.clear()

    def _handle_run(self):
        dur = self.run_dur.text().strip()
        if dur:
            self._send_serial(f"CMD:RUN:{dur}\n")
        else:
            self._send_serial("CMD:RUN\n")

    def _set_sp(self, kind):
        if kind == 'pos':
            self.sp_pos = self.sp_pos_sb.value()
            self._send_serial(f"CMD:SP_POS:{self.sp_pos:.3f}\n")
        elif kind == 'vel':
            self.sp_vel = self.sp_vel_sb.value()
            self._send_serial(f"CMD:SP_VEL:{self.sp_vel:.3f}\n")
        elif kind == 'torque':
            self.sp_torque = self.sp_torque_sb.value()
            self._send_serial(f"CMD:SP_TOR:{self.sp_torque:.3f}\n")

    # ─── DATA ─────────────────────────────────────────────────
    def _on_data(self, d):
        for k in ['pvdd','iq','id','i3','m_rpm','red_rpm','ntc','m_ang','red_ang','a','b','c']:
            if k in d:
                self.buf[k].append(d[k])

        # Push current setpoints into their buffers so they plot as flat reference lines
        self.buf['sp_pos'].append(self.sp_pos)
        self.buf['sp_vel'].append(self.sp_vel)
        self.buf['sp_torque'].append(self.sp_torque)

        # Dashboard cards
        self.card_pvdd.update(d['pvdd'],      ".2f")
        self.card_iq.update(abs(d['iq']),     ".2f")
        self.card_id.update(abs(d['id']),     ".2f")
        self.card_i3.update(abs(d['i3']),     ".2f")
        self.card_rpm.update(d['m_rpm'],      ".1f")
        self.card_redrpm.update(d['red_rpm'], ".2f")
        self.card_ntc.update(d['ntc'],        ".1f")
        self.card_state.lbl_val.setText(state_str)

        nf_ok = d['nfault'] == 1
        oc_ok = d['noctw']  == 1
        self.lbl_id.setText(f"ID: {d['ID']}")
        self.lbl_nfault.setText(f"NFault: {'OK' if nf_ok else 'FAULT'}")
        self.lbl_nfault.setStyleSheet(
            f"color:{'#00d4aa' if nf_ok else '#ff4d6d'}; font-family:'Consolas'; padding:4px 12px;")
        self.lbl_noctw.setText(f"OC Warn: {'OK' if oc_ok else 'WARN'}")
        self.lbl_noctw.setStyleSheet(
            f"color:{'#00d4aa' if oc_ok else '#ffb347'}; font-family:'Consolas'; padding:4px 12px;")
        self.lbl_ang.setText(f"Motor angle: {d['m_ang']:.2f}°")
        self.lbl_redang.setText(f"Reducer angle: {d['red_ang']:.2f}°")

        # CSV recording
        if self.recording and self.rec_writer:
            self.rec_writer.writerow([
                datetime.utcnow().isoformat(),
                d['pvdd'], d['iq'], d['id'], d['i3'],
                d['a'], d['b'], d['c'],
                d['nfault'], d['noctw'], d['ntc'],
                d['m_ang'], d['red_ang'],
                d['m_rpm'], d['red_rpm'],
                self.sp_pos, self.sp_vel, self.sp_torque
            ])

    def _on_raw(self, line):
        if self.chk_raw.isChecked():
            self.console.append(line)
            if not self.chk_pause.isChecked():
                sb = self.console.verticalScrollBar()
                sb.setValue(sb.maximum())

    def _on_recoverable_error(self, msg):
        self._status(f"⚠ {msg}", "warn")
        self.console.append(f"[WARN] {msg}")

    def _on_fatal_error(self, msg):
        self._status(f"✖ {msg}", "err")
        self.console.append(f"[FATAL] {msg}")
        self._disconnect()

    # ─── PLOTS ────────────────────────────────────────────────
    def _refresh_plots(self):
        arr = {k: np.array(self.buf[k]) for k in self.buf}
        x = self.x

        # ── Control tab plots ─────────────────────────────────
        self.crv_ctrl_pos[0].setData(x, arr['red_ang'])
        self.crv_ctrl_pos[1].setData(x, arr['sp_pos'])

        self.crv_ctrl_vel[0].setData(x, arr['m_rpm'])
        self.crv_ctrl_vel[1].setData(x, arr['sp_vel'])

        self.crv_ctrl_tor[0].setData(x, arr['iq'])
        self.crv_ctrl_tor[1].setData(x, arr['sp_torque'])
        self.crv_ctrl_tor[2].setData(x, arr['id'])

        # ── Dashboard ─────────────────────────────────────────
        self.crv_dash_rpm[0].setData(x, arr['m_rpm'])
        self.crv_dash_rpm[1].setData(x, arr['red_rpm'])

        # ── Currents ──────────────────────────────────────────
        self.crv_cur[0].setData(x, arr['iq'])
        self.crv_cur[1].setData(x, arr['id'])
        self.crv_cur[2].setData(x, arr['i3'])
        self.crv_raw[0].setData(x, arr['a'])
        self.crv_raw[1].setData(x, arr['b'])
        self.crv_raw[2].setData(x, arr['c'])

        # ── Mechanical ────────────────────────────────────────
        self.crv_rpm[0].setData(x, arr['m_rpm'])
        self.crv_rpm[1].setData(x, arr['red_rpm'])
        self.crv_ang[0].setData(x, arr['m_ang'])
        self.crv_ang[1].setData(x, arr['red_ang'])

        # ── Power ─────────────────────────────────────────────
        self.crv_pvdd[0].setData(x, arr['pvdd'])
        self.crv_ntc[0].setData(x, arr['ntc'])

    # ─── CSV RECORDING ────────────────────────────────────────
    def _toggle_record(self):
        if not self.recording:
            path, _ = QFileDialog.getSaveFileName(
                self, "Save CSV",
                f"telem_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                "CSV (*.csv)")
            if not path:
                return
            self.rec_file   = open(path, 'w', newline='')
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow([
                'timestamp','pvdd','iq','id','i3',
                'a','b','c','nfault','noctw','ntc',
                'm_ang','red_ang','m_rpm','red_rpm',
                'sp_pos','sp_vel','sp_torque'
            ])
            self.recording = True
            self.btn_record.setText("⏹  Stop Recording")
            self._status(f"Recording → {path}", "ok")
        else:
            self.recording = False
            if self.rec_file:
                self.rec_file.close()
                self.rec_file = self.rec_writer = None
            self.btn_record.setText("⏺  Save CSV")
            self._status("Recording stopped", "warn")

    def _status(self, msg, level="ok"):
        colors = {"ok": "#00d4aa", "warn": "#ffb347", "err": "#ff4d6d"}
        self.status.showMessage(f"  {msg}")
        self.status.setStyleSheet(
            f"background:#080b10; color:{colors.get(level,'#c8cdd8')};"
            "border-top:1px solid #1a2030; font-family:'Consolas'; font-size:12px;")

    def closeEvent(self, e):
        if self.worker:
            self.worker.stop()
        if self.rec_file:
            self.rec_file.close()
        e.accept()


# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("ASTURION GUI Monitor")
    try:
        app.setWindowIcon(QIcon("Flag_of_Asturias.svg.png"))
    except Exception:
        pass
    win = MotorMonitor()
    win.show()
    sys.exit(app.exec())