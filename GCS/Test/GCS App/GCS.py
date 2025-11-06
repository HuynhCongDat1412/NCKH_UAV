import sys
import json
import time
from datetime import datetime
import os
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget,
    QLabel, QComboBox, QPushButton, QDoubleSpinBox, QSpinBox,
    QGroupBox, QFormLayout, QTextEdit, QInputDialog, QSplitter
)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt, QSettings
from PyQt5.QtGui import QFont
import pyqtgraph as pg

CONFIG_FILE = "gcs_config.json"

class SerialThread(QThread):
    data_received = pyqtSignal(str)
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True
    def run(self):
        while self.running and self.port.is_open:
            try:
                if self.port.in_waiting > 0:
                    line = self.port.readline().decode('utf-8', errors='replace').rstrip()
                    if line:
                        self.data_received.emit(line)
                else:
                    time.sleep(0.001)
            except:
                time.sleep(0.01)
    def stop(self):
        self.running = False

class GCSWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial = None
        self.thread = None
        self.configs = self.load_configs()
        self.settings = QSettings("DroneGCS", "UI")
        self.dark_mode = self.settings.value("dark_mode", False, type=bool)
        self.show_uart = False
        self.last_telemetry_time = 0
        self.initUI()
        self.apply_theme()

    def initUI(self):
        self.setWindowTitle("Drone GCS")
        self.setGeometry(100, 100, 1400, 780)
        central = QWidget()
        main_layout = QVBoxLayout()
        central.setLayout(main_layout)
        self.setCentralWidget(central)

        # === TOP BAR: CANH LỀ TRÁI & PHẢI ===
        top_layout = QHBoxLayout()
        top_layout.setSpacing(12)
        top_layout.setContentsMargins(10, 8, 10, 8)

        # LEFT: COM + Baud + Connect
        left_group = QHBoxLayout()
        self.port_cb = QComboBox(); self.port_cb.setMinimumWidth(120)
        self.baud_cb = QComboBox(); self.baud_cb.addItems(["115200", "57600", "9600"])
        self.baud_cb.setCurrentText("115200")
        self.conn_btn = QPushButton("Connect")
        self.conn_btn.clicked.connect(self.toggle_connection)

        left_group.addWidget(QLabel("COM:"))
        left_group.addWidget(self.port_cb)
        left_group.addWidget(QLabel("Baud:"))
        left_group.addWidget(self.baud_cb)
        left_group.addWidget(self.conn_btn)

        top_layout.addLayout(left_group)

        # RIGHT: Status + Theme (canh phải)
        right_group = QHBoxLayout()
        right_group.setSpacing(15)
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold; min-width: 100px;")
        self.theme_btn = QPushButton("Dark Mode" if self.dark_mode else "Light Mode")
        self.theme_btn.clicked.connect(self.toggle_theme)

        right_group.addWidget(self.status_label)
        right_group.addWidget(self.theme_btn)
        top_layout.addStretch()
        top_layout.addLayout(right_group)

        main_layout.addLayout(top_layout)

        # === CONTENT ===
        content = QHBoxLayout()

        # LEFT: Telemetry
        left_col = QVBoxLayout()
        tele_box = QGroupBox("Telemetry")
        tele_layout = QFormLayout()
        tele_layout.setLabelAlignment(Qt.AlignRight)
        tele_layout.setVerticalSpacing(10)

        self.labels = {}
        for key in ["timestamp", "roll", "pitch", "yaw", "battery", "armed"]:
            lbl = QLabel("---")
            lbl.setFont(QFont("Segoe UI", 11))
            self.labels[key] = lbl
            tele_layout.addRow(f"<b>{key.replace('_', ' ').title()}:</b>", lbl)
        tele_box.setLayout(tele_layout)
        left_col.addWidget(tele_box)
        content.addLayout(left_col, 2)

        # RIGHT: Tuning + Commands
        right_col = QVBoxLayout()

        # Tuning
        tune_box = QGroupBox("Tuning Parameters")
        tune_layout = QFormLayout()
        tune_layout.setVerticalSpacing(8)
        self.p_spin = QDoubleSpinBox(); self.p_spin.setRange(0, 100); self.p_spin.setDecimals(3); self.p_spin.setValue(1.5)
        self.i_spin = QDoubleSpinBox(); self.i_spin.setRange(0, 10); self.i_spin.setDecimals(3); self.i_spin.setValue(0.05)
        self.d_spin = QDoubleSpinBox(); self.d_spin.setRange(0, 10); self.d_spin.setDecimals(3); self.d_spin.setValue(0.08)
        self.bank_spin = QSpinBox(); self.bank_spin.setRange(10, 60); self.bank_spin.setValue(30)
        tune_layout.addRow("P:", self.p_spin); tune_layout.addRow("I:", self.i_spin)
        tune_layout.addRow("D:", self.d_spin); tune_layout.addRow("Max Bank (°):", self.bank_spin)

        btn_layout = QHBoxLayout()
        self.send_tune_btn = QPushButton("Send"); self.send_tune_btn.clicked.connect(self.send_tuning)
        self.save_cfg_btn = QPushButton("Save Config"); self.save_cfg_btn.clicked.connect(self.save_current_config)
        btn_layout.addWidget(self.send_tune_btn); btn_layout.addWidget(self.save_cfg_btn)
        tune_layout.addRow(btn_layout)

        cfg_layout = QHBoxLayout()
        self.cfg_combo = QComboBox(); self.cfg_combo.addItems(self.configs.keys())
        self.load_cfg_btn = QPushButton("Load"); self.load_cfg_btn.clicked.connect(self.load_selected_config)
        self.del_cfg_btn = QPushButton("Delete"); self.del_cfg_btn.clicked.connect(self.delete_config)
        cfg_layout.addWidget(QLabel("Config:")); cfg_layout.addWidget(self.cfg_combo)
        cfg_layout.addWidget(self.load_cfg_btn); cfg_layout.addWidget(self.del_cfg_btn)
        tune_layout.addRow(cfg_layout)
        tune_box.setLayout(tune_layout)
        right_col.addWidget(tune_box)

        # Commands
        cmd_box = QGroupBox("Commands")
        cmd_layout = QFormLayout()
        cmd_layout.setVerticalSpacing(8)
        self.mode_cb = QComboBox(); self.mode_cb.addItems(["Manual", "Stabilize", "AltHold", "Loiter", "RTL"])
        self.send_mode_btn = QPushButton("Send Mode")
        self.send_mode_btn.clicked.connect(self.send_mode)
        cmd_layout.addRow("Mode:", self.mode_cb)
        cmd_layout.addRow(self.send_mode_btn)

        arm_layout = QHBoxLayout()
        self.arm_btn = QPushButton("ARM"); self.arm_btn.clicked.connect(lambda: self.send_command("ARM"))
        self.disarm_btn = QPushButton("DISARM"); self.disarm_btn.clicked.connect(lambda: self.send_command("DISARM"))
        arm_layout.addWidget(self.arm_btn); arm_layout.addWidget(self.disarm_btn)
        cmd_layout.addRow(arm_layout)

        calib_layout = QHBoxLayout()
        self.calib_gyro_btn = QPushButton("Calib Gyro"); self.calib_gyro_btn.clicked.connect(lambda: self.send_command("CALIB_GYRO"))
        self.calib_acc_btn = QPushButton("Calib Acc"); self.calib_acc_btn.clicked.connect(lambda: self.send_command("CALIB_ACC"))
        calib_layout.addWidget(self.calib_gyro_btn); calib_layout.addWidget(self.calib_acc_btn)
        cmd_layout.addRow(calib_layout)
        cmd_box.setLayout(cmd_layout)
        right_col.addWidget(cmd_box)
        content.addLayout(right_col, 1)
        main_layout.addLayout(content)

        # === BOTTOM: LOG + PLOT ===
        bottom_splitter = QSplitter(Qt.Horizontal)

        # Log + Controls
        log_container = QWidget()
        log_layout = QVBoxLayout()
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_layout.setSpacing(5)

        log_btn_layout = QHBoxLayout()
        self.uart_log_btn = QPushButton("UART Log: OFF")
        self.uart_log_btn.clicked.connect(self.toggle_uart_log)
        self.clear_log_btn = QPushButton("Clear Log")
        self.clear_log_btn.clicked.connect(self.clear_log)
        log_btn_layout.addWidget(self.uart_log_btn)
        log_btn_layout.addWidget(self.clear_log_btn)
        log_btn_layout.addStretch()

        log_box = QGroupBox("Command Log")
        log_inner = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        log_inner.addWidget(self.log_text)
        log_box.setLayout(log_inner)

        log_layout.addLayout(log_btn_layout)
        log_layout.addWidget(log_box)
        log_container.setLayout(log_layout)
        bottom_splitter.addWidget(log_container)

        # Attitude Plot
        attitude_box = QGroupBox("Attitude (Roll/Pitch/Yaw)")
        attitude_layout = QVBoxLayout()
        self.attitude_plot = pg.PlotWidget()
        self.attitude_plot.addLegend()
        self.roll_curve = self.attitude_plot.plot(pen='r', name='Roll')
        self.pitch_curve = self.attitude_plot.plot(pen='g', name='Pitch')
        self.yaw_curve = self.attitude_plot.plot(pen='b', name='Yaw')
        self.attitude_data = {'roll': [], 'pitch': [], 'yaw': []}
        attitude_layout.addWidget(self.attitude_plot)
        attitude_box.setLayout(attitude_layout)
        bottom_splitter.addWidget(attitude_box)

        main_layout.addWidget(bottom_splitter, 1)

        # Timer
        self.timer = QTimer(); self.timer.timeout.connect(self.update_ui); self.timer.start(50)
        self.refresh_ports()
        self.load_last_config()

    def refresh_ports(self):
        self.port_cb.clear()
        for p in serial.tools.list_ports.comports():
            self.port_cb.addItem(p.device)

    def toggle_uart_log(self):
        self.show_uart = not self.show_uart
        self.uart_log_btn.setText(f"UART Log: {'ON' if self.show_uart else 'OFF'}")
        self.log(f"UART Log: {'ON' if self.show_uart else 'OFF'}")

    def clear_log(self):
        self.log_text.clear()
        self.log("Log cleared")

    def connect(self):
        try:
            port = self.port_cb.currentText()
            baud = int(self.baud_cb.currentText())
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.serial.flushInput()
            self.serial.flushOutput()

            self.serial.write(b"CONNECT\n")
            self.serial.flush()

            if self.wait_for_response("CONNECT_OK", timeout=2.0):
                self.thread = SerialThread(self.serial)
                self.thread.data_received.connect(self.on_serial_data)
                self.thread.start()
                self.status_label.setText("Connected")
                self.status_label.setStyleSheet("color: green; font-weight: bold")
                self.conn_btn.setText("Disconnect")
                self.log("Connected to GCS_ESP32")
                self.last_telemetry_time = time.time()
            else:
                self.serial.close()
                self.log("CONNECT failed")
                self.status_label.setText("Disconnected")
                self.status_label.setStyleSheet("color: red; font-weight: bold")

        except Exception as e:
            self.log(f"Error: {e}")
            if self.serial and self.serial.is_open:
                self.serial.close()

    def wait_for_response(self, expected, timeout=2.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if self.show_uart:
                    self.log(f"[UART] {line}")
                if expected in line:
                    return True
            time.sleep(0.01)
        return False

    def disconnect(self):
        if self.thread: self.thread.stop(); self.thread.wait()
        if self.serial: self.serial.close()
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold")
        self.conn_btn.setText("Connect")
        self.log("Disconnected")

    def on_serial_data(self, data):
        if self.show_uart:
            self.log(f"[UART IN] {data}")
        try:
            if '{' in data and '}' in data:
                start = data.index('{')
                end = data.rindex('}') + 1
                json_str = data[start:end]
                payload = json.loads(json_str)
                self.last_telemetry_time = time.time()
                self.update_telemetry(payload)
        except Exception as e:
            if self.show_uart:
                self.log(f"[Parse Error] {e}")

    def update_telemetry(self, data):
        ts = datetime.fromtimestamp(data.get("timestamp", 0)/1000).strftime("%H:%M:%S")
        self.labels["timestamp"].setText(ts)

        roll = data.get('roll', 0); self.labels["roll"].setText(f"{roll:+.1f}°"); self.set_color(self.labels["roll"], abs(roll) < 45)
        pitch = data.get('pitch', 0); self.labels["pitch"].setText(f"{pitch:+.1f}°"); self.set_color(self.labels["pitch"], abs(pitch) < 45)
        yaw = data.get('yaw', 0); self.labels["yaw"].setText(f"{yaw:+.1f}°")
        batt = data.get("battery", 0); percent = int((batt - 10.0)/(12.6-10.0)*100); percent = max(0, min(100, percent))
        self.labels["battery"].setText(f"{batt:.1f}V ({percent}%)"); self.set_color(self.labels["battery"], batt > 11.0)
        armed = data.get("armed", False); self.labels["armed"].setText("ARMED" if armed else "DISARMED"); self.set_color(self.labels["armed"], not armed)

        self.attitude_data['roll'].append(roll)
        self.attitude_data['pitch'].append(pitch)
        self.attitude_data['yaw'].append(yaw)
        for k in self.attitude_data:
            if len(self.attitude_data[k]) > 200:
                self.attitude_data[k] = self.attitude_data[k][-200:]
        self.roll_curve.setData(self.attitude_data['roll'])
        self.pitch_curve.setData(self.attitude_data['pitch'])
        self.yaw_curve.setData(self.attitude_data['yaw'])

    def set_color(self, label, safe):
        label.setStyleSheet(f"color: {'#2e8b27' if safe else '#dc143c'}; font-weight: bold;")

    def send_tuning(self):
        cmd = {"type": "PID", "p": self.p_spin.value(), "i": self.i_spin.value(), "d": self.d_spin.value(), "max_bank": self.bank_spin.value()}
        self.send_json(cmd)
        self.log(f"→ PID: P={cmd['p']}, I={cmd['i']}, D={cmd['d']}, Bank={cmd['max_bank']}°")

    def send_command(self, cmd):
        payload = {"type": "CMD", "cmd": cmd}
        self.send_json(payload)
        self.log(f"→ CMD: {cmd}")

    def send_mode(self):
        mode = self.mode_cb.currentText()
        payload = {"type": "CMD", "cmd": "SET_MODE", "mode": mode}
        self.send_json(payload)
        self.log(f"→ MODE: {mode}")

    def send_json(self, data):
        if self.serial and self.serial.is_open:
            try:
                msg = json.dumps(data) + "\n"
                self.serial.write(msg.encode())
                if self.show_uart:
                    self.log(f"[UART OUT] {msg.strip()}")
            except:
                self.log("Send failed")

    def toggle_connection(self):
        if self.serial and self.serial.is_open:
            self.disconnect()
        else:
            self.connect()

    def toggle_theme(self):
        self.dark_mode = not self.dark_mode
        self.theme_btn.setText("Dark Mode" if self.dark_mode else "Light Mode")
        self.settings.setValue("dark_mode", self.dark_mode)
        self.apply_theme()

    def apply_theme(self):
        if self.dark_mode:
            ss = """
                * { background-color: #1e1e1e; color: #ffffff; font-size: 10pt; }
                QGroupBox { border: 1px solid #444; padding-top: 10px; }
                QFormLayout { margin: 5px; }
                QLabel { padding: 2px; }
                QPushButton { padding: 6px 12px; }
            """
        else:
            ss = """
                * { background-color: #f5f5f5; color: #000000; font-size: 10pt; }
                QGroupBox { border: 1px solid #ccc; padding-top: 10px; }
                QFormLayout { margin: 5px; }
                QLabel { padding: 2px; }
                QPushButton { padding: 6px 12px; }
            """
        self.setStyleSheet(ss)

    def log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def update_ui(self):
        if not (self.serial and self.serial.is_open):
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold")
            return

        elapsed = time.time() - self.last_telemetry_time
        if elapsed > 0.2:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold")
            if elapsed > 0.3:
                self.log("Lost connection to Drone_ESP32 (timeout 0.2s)")
        else:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green; font-weight: bold")

    def load_last_config(self):
        if "default" in self.configs:
            cfg = self.configs["default"]
            self.p_spin.setValue(cfg.get("p", 1.5)); self.i_spin.setValue(cfg.get("i", 0.05))
            self.d_spin.setValue(cfg.get("d", 0.08)); self.bank_spin.setValue(cfg.get("bank", 30))
            self.cfg_combo.setCurrentText("default")

    def save_current_config(self):
        name, ok = QInputDialog.getText(self, "Save Config", "Config name:", text="default")
        if ok and name:
            self.configs[name] = {
                "p": self.p_spin.value(), "i": self.i_spin.value(),
                "d": self.d_spin.value(), "bank": self.bank_spin.value(),
            }
            self.save_configs()
            self.cfg_combo.clear(); self.cfg_combo.addItems(self.configs.keys())
            self.cfg_combo.setCurrentText(name)
            self.log(f"Config '{name}' saved")

    def load_selected_config(self):
        name = self.cfg_combo.currentText()
        cfg = self.configs.get(name, {})
        self.p_spin.setValue(cfg.get("p", 1.5)); self.i_spin.setValue(cfg.get("i", 0.05))
        self.d_spin.setValue(cfg.get("d", 0.08)); self.bank_spin.setValue(cfg.get("bank", 30))
        self.log(f"Loaded config: {name}")

    def delete_config(self):
        name = self.cfg_combo.currentText()
        if name in self.configs and name != "default":
            del self.configs[name]; self.save_configs()
            self.cfg_combo.clear(); self.cfg_combo.addItems(self.configs.keys())
            self.log(f"Deleted: {name}")

    def load_configs(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    return json.load(f)
            except: pass
        return {"default": {"p": 1.5, "i": 0.05, "d": 0.08, "bank": 30}}

    def save_configs(self):
        with open(CONFIG_FILE, 'w') as f:
            json.dump(self.configs, f, indent=2)

    def closeEvent(self, event):
        self.disconnect()
        self.settings.setValue("dark_mode", self.dark_mode)
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = GCSWindow()
    win.show()
    sys.exit(app.exec_())