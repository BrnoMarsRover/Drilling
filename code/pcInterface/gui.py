"""
gui.py

Tkinter GUI for the drilling HMI.
All serial communication goes through the SerialWorker — the GUI never
touches the serial port directly.

Layout (4 columns, 2 content rows):
  Col 0: Drill status + Manual controls
  Col 1 row 1: Auto drilling + Height measurement
  Col 1 row 2: Sample handling
  Col 2 row 1: Device status
  Col 2 row 2: Calibration
  Col 3 rows 1-2: Log
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import protocol

POLL_INTERVAL_MS       = 100
STATE_POLL_INTERVAL_MS = 500


class App(tk.Tk):
    def __init__(self, worker):
        super().__init__()
        self.worker = worker
        self.title("Drilling HMI")
        self.resizable(False, False)

        self._build_connection_bar()
        self._build_status_panel()
        self._build_manual_controls()
        self._build_auto_controls()
        self._build_sample_controls()
        self._build_device_status_panel()
        self._build_calibration_panel()
        self._build_log()

        self._poll_rx()
        self._poll_state()

    # ------------------------------------------------------------------ #
    #  Connection bar                                                      #
    # ------------------------------------------------------------------ #

    def _build_connection_bar(self):
        frame = ttk.LabelFrame(self, text="Connection")
        frame.grid(row=0, column=0, columnspan=4, padx=8, pady=6, sticky="ew")

        ttk.Label(frame, text="COM port:").grid(row=0, column=0, padx=4, pady=4)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_btn = ttk.Button(frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=4)
        self.conn_status = ttk.Label(frame, text="Disconnected", foreground="red")
        self.conn_status.grid(row=0, column=4, padx=8)
        self._refresh_ports()

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.worker.is_connected():
            self.worker.disconnect()
            self.connect_btn.config(text="Connect")
            self.conn_status.config(text="Disconnected", foreground="red")
            self._log("Disconnected.")
        else:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "No COM port selected.")
                return
            ok = self.worker.connect(port)
            if ok:
                self.connect_btn.config(text="Disconnect")
                self.conn_status.config(text=f"Connected ({port})", foreground="green")
                self._log(f"Connected on {port}.")
            else:
                self._log(f"Failed to connect on {port}.")

    # ------------------------------------------------------------------ #
    #  Status panel (STATE response)  — column 0                          #
    # ------------------------------------------------------------------ #

    def _build_status_panel(self):
        frame = ttk.LabelFrame(self, text="Drill status  (auto-refreshed)")
        frame.grid(row=1, column=0, padx=8, pady=6, sticky="nsew")

        labels = [
            ("Carriage depth",         "carriage_depth_var",  "mm"),
            ("Depth under rover",      "depth_under_rover_var","mm"),
            ("Depth under surface",    "depth_under_surface_var","mm"),
            ("Vertical speed (actual)","vert_speed_actual_var","mm/s"),
            ("Stepper current",        "current_var",         "A"),
            ("Motor speed",            "rpm_var",             "RPM"),
            ("Motor temp",             "temp_var",            "°C"),
            ("Tray angle",             "tray_var",            "°"),
            ("State",                  "sw_state_var",        ""),
        ]

        for i, (label, attr, unit) in enumerate(labels):
            ttk.Label(frame, text=label + ":").grid(row=i, column=0, sticky="w", padx=6, pady=2)
            var = tk.StringVar(value="—")
            setattr(self, attr, var)
            ttk.Label(frame, textvariable=var, width=22, anchor="w").grid(row=i, column=1, padx=4)
            if unit:
                ttk.Label(frame, text=unit).grid(row=i, column=2, sticky="w")

        ttk.Separator(frame, orient="horizontal").grid(
            row=len(labels), column=0, columnspan=3, sticky="ew", pady=4)

        # Weight displays
        ttk.Label(frame, text="Weight (deep):").grid(
            row=len(labels)+1, column=0, sticky="w", padx=6, pady=2)
        self.weight_deep_var = tk.StringVar(value="—")
        ttk.Label(frame, textvariable=self.weight_deep_var, width=22, anchor="w").grid(
            row=len(labels)+1, column=1, padx=4)
        ttk.Label(frame, text="g").grid(row=len(labels)+1, column=2, sticky="w")

        ttk.Label(frame, text="Weight (surface):").grid(
            row=len(labels)+2, column=0, sticky="w", padx=6, pady=2)
        self.weight_surface_var = tk.StringVar(value="—")
        ttk.Label(frame, textvariable=self.weight_surface_var, width=22, anchor="w").grid(
            row=len(labels)+2, column=1, padx=4)
        ttk.Label(frame, text="g").grid(row=len(labels)+2, column=2, sticky="w")

    # ------------------------------------------------------------------ #
    #  Manual controls — column 0                                          #
    # ------------------------------------------------------------------ #

    def _build_manual_controls(self):
        frame = ttk.LabelFrame(self, text="Manual controls")
        frame.grid(row=2, column=0, padx=8, pady=6, sticky="nsew")

        ttk.Label(frame, text="Drill speed (RPM):").grid(row=0, column=0, sticky="w", padx=6, pady=3)
        self.drill_speed_var = tk.StringVar(value="0")
        ttk.Entry(frame, textvariable=self.drill_speed_var, width=8).grid(row=0, column=1, padx=4)
        ttk.Button(frame, text="Set", command=self._set_drill_speed).grid(row=0, column=2, padx=4)

        ttk.Label(frame, text="Vertical speed (mm/s):").grid(row=1, column=0, sticky="w", padx=6, pady=3)
        self.vert_speed_request_var = tk.StringVar(value="0")
        ttk.Entry(frame, textvariable=self.vert_speed_request_var, width=8).grid(row=1, column=1, padx=4)
        ttk.Button(frame, text="Set", command=self._set_vertical_speed).grid(row=1, column=2, padx=4)

        ttk.Label(frame, text="Storage position:").grid(row=2, column=0, sticky="w", padx=6, pady=3)
        self.storage_pos_var = tk.StringVar(value="0")
        ttk.Entry(frame, textvariable=self.storage_pos_var, width=8).grid(row=2, column=1, padx=4)
        ttk.Button(frame, text="Set", command=self._set_storage_position).grid(row=2, column=2, padx=4)

        ttk.Separator(frame, orient="horizontal").grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=6)

        ttk.Button(frame, text="Calibrate carriage depth", command=self._calibrate_height).grid(
            row=4, column=0, columnspan=3, sticky="ew", padx=6, pady=2)
        ttk.Button(frame, text="RESTART", command=self._restart).grid(
            row=5, column=0, columnspan=3, sticky="ew", padx=6, pady=2)

    # ------------------------------------------------------------------ #
    #  Automatic drilling — column 1                                       #
    # ------------------------------------------------------------------ #

    def _build_auto_controls(self):
        frame = ttk.LabelFrame(self, text="Automatic drilling")
        frame.grid(row=1, column=1, padx=8, pady=6, sticky="nsew")

        ttk.Label(frame, text="Target depth (cm):").grid(row=0, column=0, sticky="w", padx=6, pady=3)
        self.drill_depth_var = tk.StringVar(value="10")
        ttk.Entry(frame, textvariable=self.drill_depth_var, width=8).grid(row=0, column=1, padx=4)

        ttk.Button(frame, text="Start auto drill", command=self._drill_auto).grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=6, pady=4)
        ttk.Button(frame, text="Stop auto drill", command=self._stop_auto).grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=6, pady=2)

        ttk.Separator(frame, orient="horizontal").grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=6)

        ttk.Label(frame, text="Height above ground:").grid(
            row=4, column=0, sticky="w", padx=6, pady=2)
        self.height_above_ground_var = tk.StringVar(value="—")
        ttk.Label(frame, textvariable=self.height_above_ground_var, width=10, anchor="w").grid(
            row=4, column=1, padx=4)
        ttk.Label(frame, text="mm").grid(row=4, column=2, sticky="w")

        ttk.Button(frame, text="Measure height above ground", command=self._measure_height).grid(
            row=5, column=0, columnspan=3, sticky="ew", padx=6, pady=2)
        ttk.Button(frame, text="Get height above ground", command=self._get_height).grid(
            row=6, column=0, columnspan=3, sticky="ew", padx=6, pady=2)

    # ------------------------------------------------------------------ #
    #  Sample handling — column 1                                          #
    # ------------------------------------------------------------------ #

    def _build_sample_controls(self):
        frame = ttk.LabelFrame(self, text="Sample handling")
        frame.grid(row=2, column=1, padx=8, pady=6, sticky="nsew")

        buttons = [
            ("Weigh deep sample",    self._weigh_deep),
            ("Weigh surface sample", self._weigh_surface),
            ("Get deep weight",      self._get_weight_deep),
            ("Get surface weight",   self._get_weight_surface),
            ("Rock box — open",      self._rock_open),
            ("Rock box — close",     self._rock_close),
            ("Sand box — open",      self._sand_open),
            ("Sand box — close",     self._sand_close),
            ("Storage — set hold",   self._set_hold_mode),
            ("Storage — clear hold", self._clear_hold_mode),
        ]
        for i, (label, cmd) in enumerate(buttons):
            ttk.Button(frame, text=label, command=cmd).grid(
                row=i, column=0, columnspan=2, sticky="ew", padx=6, pady=2)



    # ------------------------------------------------------------------ #
    #  Device status — column 2, row 1                                     #
    # ------------------------------------------------------------------ #

    def _build_device_status_panel(self):
        frame = ttk.LabelFrame(self, text="Device status")
        frame.grid(row=1, column=2, padx=8, pady=6, sticky="nsew")

        ttk.Button(frame, text="Check devices", command=self._start_dev_check).grid(
            row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=4)
        ttk.Button(frame, text="Get device status", command=self._get_device_status).grid(
            row=1, column=0, columnspan=2, sticky="ew", padx=6, pady=2)

        ttk.Separator(frame, orient="horizontal").grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=6)

        self._device_indicators = []
        for i, name in enumerate(protocol.DEVICE_NAMES):
            indicator = tk.Label(frame, text="●", foreground="grey", font=("TkDefaultFont", 12))
            indicator.grid(row=3+i, column=0, padx=(6, 2), pady=1)
            ttk.Label(frame, text=name, anchor="w").grid(row=3+i, column=1, sticky="w", padx=(0, 6))
            self._device_indicators.append(indicator)

    def _update_device_indicators(self, devices: list):
        for i, device in enumerate(devices):
            color = "green" if device["ok"] else "red"
            self._device_indicators[i].config(foreground=color)

    # ------------------------------------------------------------------ #
    #  Calibration — column 3, row 2                                       #
    # ------------------------------------------------------------------ #

    def _build_calibration_panel(self):
        frame = ttk.LabelFrame(self, text="Calibration")
        frame.grid(row=2, column=2, padx=8, pady=6, sticky="new")

        ttk.Label(frame, text="Calibration weight (g):").grid(
            row=0, column=0, sticky="w", padx=6, pady=2)
        self.calib_weight_var = tk.StringVar(value="100.0")
        ttk.Entry(frame, textvariable=self.calib_weight_var, width=8).grid(
            row=0, column=1, padx=4)

        calib_buttons = [
            ("Cal 0g — deep",    self._calibrate_0_deep),
            ("Cal Xg — deep",    self._calibrate_x_deep),
            ("Cal 0g — surface", self._calibrate_0_surface),
            ("Cal Xg — surface", self._calibrate_x_surface),
        ]
        for i, (label, cmd) in enumerate(calib_buttons):
            ttk.Button(frame, text=label, command=cmd).grid(
                row=i+1, column=0, columnspan=2, sticky="ew", padx=6, pady=2)

    # ------------------------------------------------------------------ #
    #  Log — column 3, row 1                                               #
    # ------------------------------------------------------------------ #

    def _build_log(self):
        frame = ttk.LabelFrame(self, text="Log")
        frame.grid(row=1, column=3, rowspan=2, padx=8, pady=6, sticky="nsew")

        self.log_text = tk.Text(frame, height=30, width=50, state="disabled",
                                wrap="word", font=("Courier New", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        scrollbar = ttk.Scrollbar(frame, command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=scrollbar.set)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

    def _log(self, message: str):
        self.log_text.config(state="normal")
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ------------------------------------------------------------------ #
    #  Computed depth helpers                                              #
    # ------------------------------------------------------------------ #

    # Constants from protocol spec (mm)
    _CARRIAGE_TOP_TO_SPIRAL_TIP_MM = 720
    _LIN_AXIS_ZERO_TO_SENSOR_MM    = 775

    # Last known values — None until first data arrives
    _last_carriage_depth_mm      = None
    _last_height_above_ground_mm = None

    def _update_computed_depths(self):
        if self._last_carriage_depth_mm is None:
            return

        offset = self._last_carriage_depth_mm + self._CARRIAGE_TOP_TO_SPIRAL_TIP_MM - self._LIN_AXIS_ZERO_TO_SENSOR_MM
        self.depth_under_rover_var.set(str(offset))

        if self._last_height_above_ground_mm is not None:
            under_surface = offset - self._last_height_above_ground_mm
            self.depth_under_surface_var.set(str(under_surface))
        else:
            self.depth_under_surface_var.set("— (no height reading)")

    # ------------------------------------------------------------------ #
    #  Polling loops                                                       #
    # ------------------------------------------------------------------ #

    def _poll_rx(self):
        while not self.worker.rx_queue.empty():
            try:
                msg = self.worker.rx_queue.get_nowait()
                self._handle_message(msg)
            except Exception:
                pass
        self.after(POLL_INTERVAL_MS, self._poll_rx)

    def _poll_state(self):
        if self.worker.is_connected():
            self.worker.send(protocol.cmd_state())
        self.after(STATE_POLL_INTERVAL_MS, self._poll_state)

    def _handle_message(self, msg: dict):
        if "debug" in msg:
            self._log(f"[ESP] {msg['debug']}")
            return

        if "error" in msg:
            self._log(f"ERROR: {msg['error']}")
            return

        if msg.get("nack"):
            self._log("NACK — drill refused the command.")
            return

        code = msg.get("code")

        if code == protocol.CMD_STATE and "state" in msg:
            s = msg["state"]
            self._last_carriage_depth_mm = s["height_mm"]
            self.carriage_depth_var.set(str(s["height_mm"]))
            self._update_computed_depths()
            self.vert_speed_actual_var.set(f"{s['vert_speed']:.1f}")
            self.current_var.set(f"{s['current_a']:.2f}")
            self.rpm_var.set(str(s["rpm"]))
            self.temp_var.set(str(s["temp_c"]))
            self.tray_var.set(str(s["tray_angle"]))
            self.sw_state_var.set(s["sw_state_str"])

        elif code == protocol.CMD_GET_HEIGHT and "height_mm" in msg:
            self._last_height_above_ground_mm = msg["height_mm"]
            self.height_above_ground_var.set(str(msg["height_mm"]))
            self._update_computed_depths()
            self._log(f"Height above ground: {msg['height_mm']} mm")

        elif code == protocol.CMD_GET_WEIGHT_DEEP and "weight" in msg:
            self.weight_deep_var.set(f"{msg['weight']:.2f}")
            self._log(f"Deep sample weight: {msg['weight']:.2f} g  (raw ADC: {msg['adc_raw']})")

        elif code == protocol.CMD_GET_WEIGHT_SURFACE and "weight" in msg:
            self.weight_surface_var.set(f"{msg['weight']:.2f}")
            self._log(f"Surface sample weight: {msg['weight']:.2f} g  (raw ADC: {msg['adc_raw']})")

        elif code == protocol.CMD_GET_DEVICE_STATUS and "device_status" in msg:
            self._update_device_indicators(msg["device_status"])
            failed = [d["name"] for d in msg["device_status"] if not d["ok"]]
            if failed:
                self._log(f"Device check — failed: {', '.join(failed)}")
            else:
                self._log("Device check — all devices OK.")

        else:
            self._log(f"ACK: command 0x{code:02X}")

    # ------------------------------------------------------------------ #
    #  Command handlers                                                    #
    # ------------------------------------------------------------------ #

    def _send(self, raw: bytes, description: str):
        if not self.worker.is_connected():
            self._log("Not connected.")
            return
        self.worker.send(raw)
        self._log(f"Sent: {description}")

    def _restart(self):
        if messagebox.askyesno("Confirm", "Restart the drill controller?"):
            self._send(protocol.cmd_restart(), "RESTART")

    def _calibrate_height(self):
        self._send(protocol.cmd_calibrate_height(), "CALIBRATE HEIGHT")

    def _measure_height(self):
        self._send(protocol.cmd_measure_height(), "MEASURE HEIGHT")

    def _get_height(self):
        self._send(protocol.cmd_get_height(), "GET HEIGHT")

    def _start_dev_check(self):
        self._send(protocol.cmd_start_dev_check(), "START DEVICE CHECK")

    def _get_device_status(self):
        self._send(protocol.cmd_get_device_status(), "GET DEVICE STATUS")

    def _set_drill_speed(self):
        try:
            rpm = int(self.drill_speed_var.get())
            self._send(protocol.cmd_drill_speed(rpm), f"DRILL SPEED {rpm} RPM")
        except ValueError:
            self._log("Invalid RPM value.")

    def _set_vertical_speed(self):
        try:
            mm_s = float(self.vert_speed_request_var.get())
            if not (-12.8 <= mm_s <= 12.7):
                self._log("Vertical speed must be between -12.8 and 12.7 mm/s.")
                return
            self._send(protocol.cmd_vertical_speed(mm_s), f"VERTICAL SPEED {mm_s} mm/s")
        except ValueError:
            self._log("Invalid speed value.")

    def _set_storage_position(self):
        try:
            pos = int(self.storage_pos_var.get())
            self._send(protocol.cmd_storage_position(pos), f"STORAGE POSITION {pos}")
        except ValueError:
            self._log("Invalid position value.")

    def _drill_auto(self):
        try:
            depth = int(self.drill_depth_var.get())
            if not (0 <= depth <= 255):
                self._log("Depth must be between 0 and 255 cm.")
                return
            self._send(protocol.cmd_drill_auto(depth), f"DRILL AUTO depth={depth} cm")
        except ValueError:
            self._log("Invalid depth value.")

    def _stop_auto(self):
        self._send(protocol.cmd_stop_auto(), "STOP AUTO")

    def _weigh_deep(self):
        self._send(protocol.cmd_weigh_deep(), "WEIGH DEEP")

    def _weigh_surface(self):
        self._send(protocol.cmd_weigh_surface(), "WEIGH SURFACE")

    def _get_weight_deep(self):
        self._send(protocol.cmd_get_weight_deep(), "GET WEIGHT DEEP")

    def _get_weight_surface(self):
        self._send(protocol.cmd_get_weight_surface(), "GET WEIGHT SURFACE")

    def _calibrate_0_deep(self):
        self._send(protocol.cmd_calibrate_0_deep(), "CALIBRATE 0 DEEP")

    def _calibrate_x_deep(self):
        try:
            weight = float(self.calib_weight_var.get())
            self._send(protocol.cmd_calibrate_x_deep(weight), f"CALIBRATE X DEEP ({weight} g)")
        except ValueError:
            self._log("Invalid calibration weight.")

    def _calibrate_0_surface(self):
        self._send(protocol.cmd_calibrate_0_surface(), "CALIBRATE 0 SURFACE")

    def _calibrate_x_surface(self):
        try:
            weight = float(self.calib_weight_var.get())
            self._send(protocol.cmd_calibrate_x_surface(weight), f"CALIBRATE X SURFACE ({weight} g)")
        except ValueError:
            self._log("Invalid calibration weight.")

    def _set_hold_mode(self):
        self._send(protocol.cmd_set_hold_mode(), "SET HOLD MODE")

    def _clear_hold_mode(self):
        self._send(protocol.cmd_clear_hold_mode(), "CLEAR HOLD MODE")

    def _rock_open(self):
        self._send(protocol.cmd_rock_open(), "ROCK OPEN")

    def _rock_close(self):
        self._send(protocol.cmd_rock_close(), "ROCK CLOSE")

    def _sand_open(self):
        self._send(protocol.cmd_sand_open(), "SAND OPEN")

    def _sand_close(self):
        self._send(protocol.cmd_sand_close(), "SAND CLOSE")
