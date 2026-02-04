#!/usr/bin/env python3.12
import tkinter as tk
from tkinter import ttk
import can
import struct
import threading
import time
import csv
import os

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available. Running without ROS2 publishing.")


class TorqueVectoringUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Control Test")
        self.root.geometry("750x760")
        self.root.configure(bg="#f0f0f0")

        # Style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", font=("Arial", 12), background="#f0f0f0")
        style.configure("TButton", font=("Arial", 10))
        style.configure("TFrame", background="#f0f0f0")
        style.configure("Green.Vertical.TProgressbar", background="green")
        style.configure("SwitchStatus.TLabel", font=("Arial", 12, "bold"))
        style.configure("Rpm.Vertical.TProgressbar", background="#00aa00", troughcolor="#e6e6e6")

        # CAN bus initialization
        try:
            self.bus = can.interface.Bus(channel="can0", interface="socketcan")
            self.can_status = "CAN Connected"
        except Exception as e:
            print(f"Error initializing CAN bus: {e}")
            self.bus = None
            self.can_status = "CAN Disconnected"

        # ROS2 initialization
        if ROS2_AVAILABLE:
            rclpy.init()
            self.node = Node("torque_vectoring_ui")
            self.pub_left_torque = self.node.create_publisher(
                Float32, "left_motor_torque", 10
            )
        else:
            self.node = None

        # Variables - direct motor control (0-100%)
        self.left_torque = tk.DoubleVar(value=0.0)
        self.left_rpm_display = tk.StringVar(value="--")
        self.horizontal_torque = tk.DoubleVar(value=0.0)
        self.left_motor_rpm = tk.IntVar(value=0)
        self.left_rpm_valid = False
        self.rpm_canvases = []
        self.rpm_canvas_size = (20, 200)
        self.auto_path = tk.StringVar(value="Endurance.csv")
        self.auto_time = tk.StringVar(value="00:00")
        self.auto_max_time = "00:00"
        self.auto_throttle = tk.IntVar(value=0)
        self.auto_rpm = tk.IntVar(value=0)
        self.auto_bar_canvases = []
        self.auto_mode = tk.StringVar(value="Endurance")
        self.auto_torque = tk.DoubleVar(value=0.0)
        self.auto_running = False
        self.auto_paused = False
        self.auto_after_id = None
        self.auto_data = []
        self.auto_index = 0
        self.auto_start_time = None
        self.auto_elapsed_offset = 0.0

        # Node ID variables (CAN ID will be calculated: CAN_ID = (Packet_ID << 5) | Node_ID)
        # Packet ID for Set Current is 0x05
        # Packet ID for General Data 3 (temperatures) is 0x22
        self.SET_CURRENT_PACKET_ID = 0x05
        self.GENERAL_DATA_3_PACKET_ID = 0x22
        # Switch status CAN ID (from your screenshot)
        self.SWITCH_STATUS_CAN_ID = 0x750
        self.PUMP_SPEED_CAN_ID = 0x751
        self.MOTOR_POLE_PAIRS = 4
        self.left_node_id = tk.StringVar(value="4")

        # Input voltage display (user will populate CAN read logic)
        self.input_voltage = tk.StringVar(value="--")

        # VCU State display
        self.vcu_state = tk.StringVar(value="--")

        # Temperature variables
        self.left_controller_temp = tk.StringVar(value="--")
        self.left_motor_temp = tk.StringVar(value="--")

        # Title
        title_label = ttk.Label(
            root,
            text="Motor Control Test",
            font=("Arial", 18, "bold"),
            background="#f0f0f0",
        )
        title_label.pack(pady=10)

        # Frames
        sliders_frame = ttk.Frame(root)
        sliders_frame.pack(pady=20, padx=10, fill="both", expand=True)

        # Horizontal slider frame will be packed later after sliders_frame content

        can_id_frame = ttk.Frame(root)
        button_frame = ttk.Frame(root)
        status_frame = ttk.Frame(root)

        # Switch variables
        self.shutdown_switch = tk.BooleanVar(value=False)
        self.ignition_switch = tk.BooleanVar(value=False)
        self.r2d_switch = tk.BooleanVar(value=False)

        # Accelerator tabs (Left Motor Slider)
        self.accel_notebook = ttk.Notebook(sliders_frame)
        self.accel_notebook.pack(side="left", expand=True, padx=30)

        accel_tab_1 = ttk.Frame(self.accel_notebook)
        accel_tab_2 = ttk.Frame(self.accel_notebook)
        self.accel_notebook.add(accel_tab_1, text="Manual Control")
        self.accel_notebook.add(accel_tab_2, text="Auto Control")
        self.auto_tab = accel_tab_2

        self.build_accelerator_tab(accel_tab_1)
        self.root.update_idletasks()
        self.build_auto_tab(accel_tab_2)

        # Center switches
        switch_frame = ttk.Frame(sliders_frame)
        switch_frame.pack(side="left", expand=True, padx=50)

        ttk.Label(switch_frame, text="Switches", font=("Arial", 14, "bold")).pack(pady=5)
        shutdown_row = ttk.Frame(switch_frame)
        shutdown_row.pack(pady=6, fill="x")
        self.shutdown_button = ttk.Button(
            shutdown_row,
            text="Shutdown",
            command=lambda: self.toggle_switch(self.shutdown_switch),
            width=12,
        )
        self.shutdown_button.pack(side="left", padx=(0, 10))
        self.shutdown_status = ttk.Label(
            shutdown_row,
            text="OFF",
            style="SwitchStatus.TLabel",
            foreground="red",
        )
        self.shutdown_status.pack(side="left")

        ignition_row = ttk.Frame(switch_frame)
        ignition_row.pack(pady=6, fill="x")
        self.ignition_button = ttk.Button(
            ignition_row,
            text="Ignition",
            command=lambda: self.toggle_switch(self.ignition_switch),
            width=12,
        )
        self.ignition_button.pack(side="left", padx=(0, 10))
        self.ignition_status = ttk.Label(
            ignition_row,
            text="OFF",
            style="SwitchStatus.TLabel",
            foreground="red",
        )
        self.ignition_status.pack(side="left")

        r2d_row = ttk.Frame(switch_frame)
        r2d_row.pack(pady=6, fill="x")
        self.r2d_button = ttk.Button(
            r2d_row,
            text="R2D",
            command=lambda: self.toggle_switch(self.r2d_switch),
            width=12,
        )
        self.r2d_button.pack(side="left", padx=(0, 10))
        self.r2d_status = ttk.Label(
            r2d_row,
            text="OFF",
            style="SwitchStatus.TLabel",
            foreground="red",
        )
        self.r2d_status.pack(side="left")

        self.update_switch_styles()

        # Status box under switches: Input Voltage and VCU State
        status_box = ttk.Frame(switch_frame)
        status_box.pack(pady=20)
        ttk.Label(status_box, text="Status", font=("Arial", 14, "bold")).pack(pady=(0,6))

        status_row = ttk.Frame(status_box)
        status_row.pack(pady=2, fill="x")
        ttk.Label(status_row, text="Input Voltage:", font=("Arial", 12)).pack(side="left")
        ttk.Label(status_row, textvariable=self.input_voltage, font=("Arial", 12, "bold"), foreground="red").pack(side="left", padx=(6,6))
        ttk.Label(status_row, text="V", font=("Arial", 12)).pack(side="left")

        vcu_row = ttk.Frame(status_box)
        vcu_row.pack(pady=2, fill="x")
        ttk.Label(vcu_row, text="VCU State:", font=("Arial", 12)).pack(side="left")
        ttk.Label(vcu_row, textvariable=self.vcu_state, font=("Arial", 12, "bold")).pack(side="left", padx=(6,6))

        # Horizontal Slider Frame
        horizontal_frame = ttk.Frame(root)
        horizontal_frame.pack(pady=10, padx=10, fill="x")
        
        ttk.Label(horizontal_frame, text="Pump Speed:", font=("Arial", 12)).pack(side="left", padx=(10, 20))
        
        self.horizontal_slider = tk.Scale(
            horizontal_frame,
            from_=0,
            to=100,
            resolution=1,
            variable=self.horizontal_torque,
            orient=tk.HORIZONTAL,
            length=400,
            width=20,
            font=("Arial", 10),
            command=lambda _: self.update_display(),
        )
        self.horizontal_slider.pack(side="left", pady=5)

        # Left Motor Temperatures (next to horizontal slider)
        temp_left_frame = ttk.Frame(horizontal_frame)
        temp_left_frame.pack(side="left", padx=(30, 10))
        ttk.Label(temp_left_frame, text="Inverter:", font=("Arial", 10)).grid(row=0, column=0, sticky="e")
        ttk.Label(temp_left_frame, textvariable=self.left_controller_temp, font=("Arial", 10, "bold"), foreground="red").grid(row=0, column=1, sticky="w")
        ttk.Label(temp_left_frame, text="°C", font=("Arial", 10)).grid(row=0, column=2, sticky="w")
        ttk.Label(temp_left_frame, text="Motor:", font=("Arial", 10)).grid(row=1, column=0, sticky="e")
        ttk.Label(temp_left_frame, textvariable=self.left_motor_temp, font=("Arial", 10, "bold"), foreground="orange").grid(row=1, column=1, sticky="w")
        ttk.Label(temp_left_frame, text="°C", font=("Arial", 10)).grid(row=1, column=2, sticky="w")
        

        # Now pack the remaining frames in correct order
        can_id_frame.pack(pady=20, padx=10, fill="x")
        button_frame.pack(pady=20, padx=10, fill="x")
        status_frame.pack(pady=20, padx=10, fill="x")

        # Node ID Configuration (CAN ID = (Packet_ID << 5) | Node_ID)
        ttk.Label(can_id_frame, text="Left Motor Node ID:", font=("Arial", 12)).grid(
            row=0, column=0, padx=10, pady=5, sticky="w"
        )
        ttk.Entry(
            can_id_frame, textvariable=self.left_node_id, font=("Arial", 12), width=10
        ).grid(row=0, column=1, padx=10, pady=5, sticky="w")
        
        
        # Button Elements
        ttk.Button(
            button_frame, text="Send Once", command=self.send_torques
        ).grid(row=0, column=0, padx=10, pady=10)

        ttk.Button(
            button_frame, text="Reset to Zero", command=self.reset_sliders
        ).grid(row=0, column=1, padx=10, pady=10)

        # Status
        ttk.Label(status_frame, text="CAN Status:", font=("Arial", 12)).grid(
            row=0, column=0, padx=10, pady=5, sticky="w"
        )
        self.can_status_label = ttk.Label(
            status_frame,
            text=self.can_status,
            font=("Arial", 12, "bold"),
            foreground="green" if self.bus else "red",
        )
        self.can_status_label.grid(row=0, column=1, padx=10, pady=5, sticky="w")

        ttk.Label(status_frame, text="ROS2 Status:", font=("Arial", 12)).grid(
            row=0, column=2, padx=10, pady=5, sticky="w"
        )
        ros2_status = "Connected" if ROS2_AVAILABLE and self.node else "Disconnected"
        self.ros2_status_label = ttk.Label(
            status_frame,
            text=ros2_status,
            font=("Arial", 12, "bold"),
            foreground="green" if ROS2_AVAILABLE and self.node else "red",
        )
        self.ros2_status_label.grid(row=0, column=3, padx=10, pady=5, sticky="w")

        

        # Torque send loop (always running, gated by switches)
        self.torque_send_interval_ms = 100
        self.torque_after_id = None

        # Start CAN receive thread for temperatures
        self.receiving = True
        self.receive_thread = threading.Thread(target=self.can_receive_loop, daemon=True)
        self.receive_thread.start()

        # Start torque send loop
        self.schedule_torque_send()

        # Start switch status send loop (always sending)
        self.switch_send_interval_ms = 100
        self.switch_after_id = None
        self.schedule_switch_send()

        # Demo RPM animation (preview) removed per request

    def can_receive_loop(self):
        """Thread to receive CAN messages and update temperatures"""
        if self.bus is None:
            return
        
        while self.receiving:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is not None:
                    self.process_can_message(msg)
            except Exception as e:
                print(f"CAN receive error: {e}")

    def process_can_message(self, msg):
        """Process received CAN message and extract temperatures"""
        # (debug indicator removed)
        # Extract Node ID and Packet ID from CAN ID
        node_id = msg.arbitration_id & 0x1F
        packet_id = msg.arbitration_id >> 5
        # Try read configured left node ID for filtering (used for temperatures and voltage)
        try:
            left_node = int(self.left_node_id.get())
        except ValueError:
            left_node = None

        # Check for General Data 1 message with Packet ID 0x20
        # Bytes 0-3: ERPM (signed 32-bit, big-endian)
        # Bytes 6-7: Input voltage (signed 16-bit, big-endian), scaled by 1
        if packet_id == 0x20 and len(msg.data) >= 8:
            try:
                erpm = struct.unpack(">i", msg.data[0:4])[0]
                motor_rpm = int(erpm / self.MOTOR_POLE_PAIRS)
                self.left_motor_rpm.set(abs(motor_rpm))
                self.auto_rpm.set(abs(motor_rpm))
                self.left_rpm_valid = True
            except Exception:
                self.left_rpm_valid = False

            try:
                raw = struct.unpack(">h", msg.data[6:8])[0]
                self.set_input_voltage(raw)
            except Exception:
                # On parse error, clear the display but don't spam the console
                self.input_voltage.set("--")
        
        # Check if this is a General Data 3 message (0x22) - temperatures
        if packet_id == self.GENERAL_DATA_3_PACKET_ID and len(msg.data) >= 4:
            # Bytes 0-1: Controller temperature (signed 16-bit, big-endian, scale 10)
            # Bytes 2-3: Motor temperature (signed 16-bit, big-endian, scale 10)
            controller_temp_raw = struct.unpack(">h", msg.data[0:2])[0]
            motor_temp_raw = struct.unpack(">h", msg.data[2:4])[0]
            
            controller_temp = controller_temp_raw / 10.0
            motor_temp = motor_temp_raw / 10.0
            
            # Check which motor this belongs to (temperatures still filtered by node)
            if left_node is not None and node_id == left_node:
                self.left_controller_temp.set(f"{controller_temp:.1f}")
                self.left_motor_temp.set(f"{motor_temp:.1f}")
        # Do not filter by node id — per request, VCU state comes from any message with packet 0x22
        # Accept messages where packet_id == 0x22 OR raw arbitration id == 0x22
        if (msg.arbitration_id == 0x752) and len(msg.data) >= 5:
            try:
                state_raw = int(msg.data[4])
                self.set_vcu_state(state_raw)
            except Exception:
                # schedule UI update to clear value safely from any thread
                if self.root:
                    self.root.after(0, lambda: self.vcu_state.set("--"))
                else:
                    self.vcu_state.set("--")

    def build_accelerator_tab(self, parent):
        left_frame = ttk.Frame(parent)
        left_frame.pack(pady=10, padx=10, fill="both", expand=True)

        ttk.Label(left_frame, text="Motor", font=("Arial", 14, "bold")).pack(pady=5, anchor="w")
        ttk.Label(left_frame, text="Throttle Position      RPM", font=("Arial", 10, "bold")).pack(pady=(0, 6), anchor="w")

        slider_row = ttk.Frame(left_frame)
        slider_row.pack(pady=10)

        tk.Scale(
            slider_row,
            from_=100,
            to=0,
            resolution=0.1,
            variable=self.left_torque,
            orient=tk.VERTICAL,
            length=200,
            width=40,
            font=("Arial", 12),
            command=lambda _: self.update_display(),
        ).grid(row=0, column=0, padx=(0, 8))

        value_row = ttk.Frame(slider_row)
        value_row.grid(row=0, column=3)
        ttk.Label(value_row, text="RPM:", font=("Arial", 10, "bold")).pack(side="left", padx=(0, 6))
        ttk.Label(
            value_row,
            textvariable=self.left_rpm_display,
            font=("Arial", 16, "bold"),
            foreground="blue",
            width=5,
            anchor="e",
        ).pack(side="right")

        # RPM indicator (0-20000)
        rpm_frame = ttk.Frame(slider_row)
        rpm_frame.grid(row=0, column=2, padx=(40, 0))
        #ttk.Label(rpm_frame, text="RPM", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 4))
        ttk.Label(rpm_frame, text="20000", font=("Arial", 9)).grid(row=1, column=1, sticky="n", padx=(6, 0))
        canvas_width, canvas_height = self.rpm_canvas_size
        rpm_canvas = tk.Canvas(
            rpm_frame,
            width=canvas_width,
            height=canvas_height,
            highlightthickness=0,
            bg="#e6e6e6",
        )
        rpm_canvas.grid(row=1, column=0)

        # Solid bar matching the RPM text color
        for y in range(canvas_height):
            rpm_canvas.create_line(0, y, canvas_width, y, fill="blue")

        # Mask for empty portion (updated with RPM)
        mask_id = rpm_canvas.create_rectangle(
            0,
            0,
            canvas_width,
            canvas_height,
            fill="#e6e6e6",
            outline="",
        )
        self.rpm_canvases.append((rpm_canvas, mask_id, self.left_motor_rpm, 20000))
        ttk.Label(rpm_frame, text="0", font=("Arial", 9)).grid(row=1, column=1, sticky="s", padx=(6, 0))

    def build_auto_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        # Path entry across the top
        path_row = ttk.Frame(parent)
        path_row.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=(10, 6))
        path_row.columnconfigure(0, weight=1)
        mode_combo = ttk.Combobox(
            path_row,
            textvariable=self.auto_mode,
            state="readonly",
            values=["Acceleration", "Skidpad", "Auto X", "Endurance"],
            width=14,
            font=("Arial", 11),
        )
        mode_combo.grid(row=0, column=0, sticky="w")
        mode_combo.bind("<<ComboboxSelected>>", self.on_auto_mode_change)

        self.auto_mode_files = {
            "Acceleration": "Acceleration.csv",
            "Skidpad": "SkidPad.csv",
            "Auto X": "Auto_X.csv",
            "Endurance": "Endurance.csv",
        }

        ttk.Entry(
            path_row,
            textvariable=self.auto_path,
            font=("Arial", 11),
        ).grid(row=0, column=1, sticky="ew", padx=(10, 0))

        # Left side: time centered + buttons at the bottom
        left_panel = ttk.Frame(parent)
        left_panel.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        left_panel.rowconfigure(0, weight=1)
        left_panel.rowconfigure(1, weight=0)
        left_panel.rowconfigure(2, weight=1)
        left_panel.rowconfigure(3, weight=0)
        left_panel.columnconfigure(0, weight=1)

        time_block = ttk.Frame(left_panel)
        time_block.grid(row=1, column=0)
        ttk.Label(time_block, text="Time", font=("Arial", 12, "bold")).pack()
        ttk.Label(
            time_block,
            textvariable=self.auto_time,
            font=("Arial", 22, "bold"),
            foreground="blue",
        ).pack()

        btn_row = ttk.Frame(left_panel)
        btn_row.grid(row=3, column=0, sticky="s", pady=(10, 0))
        self.auto_start_button = ttk.Button(
            btn_row, text="Start", width=10, command=self.start_auto
        )
        self.auto_start_button.pack(side="left", padx=(0, 10))
        ttk.Button(btn_row, text="Stop", width=10, command=self.stop_auto).pack(side="left", padx=(0, 10))
        ttk.Button(btn_row, text="Reset", width=10, command=self.reset_auto).pack(side="left")

        # Right side: two bars (Throttle + RPM)
        right_panel = ttk.Frame(parent)
        right_panel.grid(row=1, column=1, sticky="n", padx=10, pady=(0, 10))

        bars_row = ttk.Frame(right_panel)
        bars_row.pack(anchor="n")

        throttle_frame = ttk.Frame(bars_row)
        throttle_frame.pack(side="left", padx=(0, 20))
        ttk.Label(throttle_frame, text="Torque", font=("Arial", 10, "bold")).pack(pady=(0, 6))
        self._create_solid_bar(throttle_frame, self.auto_torque, 100)

        rpm_bar_frame = ttk.Frame(bars_row)
        rpm_bar_frame.pack(side="left")
        ttk.Label(rpm_bar_frame, text="RPM", font=("Arial", 10, "bold")).pack(pady=(0, 6))
        self._create_solid_bar(rpm_bar_frame, self.auto_rpm, 20000)

    def _create_solid_bar(self, parent, value_var, max_value):
        canvas_width, canvas_height = self.rpm_canvas_size
        canvas = tk.Canvas(
            parent,
            width=canvas_width,
            height=canvas_height,
            highlightthickness=0,
            bg="#e6e6e6",
        )
        canvas.pack()
        for y in range(canvas_height):
            canvas.create_line(0, y, canvas_width, y, fill="blue")
        mask_id = canvas.create_rectangle(
            0,
            0,
            canvas_width,
            canvas_height,
            fill="#e6e6e6",
            outline="",
        )
        self.auto_bar_canvases.append((canvas, mask_id, value_var, max_value))

    def load_auto_csv(self, path):
        if not path:
            return []

        if not os.path.isabs(path):
            base_dir = os.path.join(os.getcwd(), "database")
            candidate = os.path.join(base_dir, path)
            if os.path.exists(candidate):
                path = candidate
            else:
                path = os.path.join(os.getcwd(), path)

        if not os.path.exists(path):
            print(f"CSV not found: {path}")
            return []

        data = []
        with open(path, "r", newline="") as f:
            reader = csv.reader(f, delimiter=";")
            rows = list(reader)

        if not rows:
            return []

        header = rows[0]
        if len(header) < 2:
            return []

        time_idx = 0
        torque_idx = 1

        for row in rows[1:]:
            if len(row) < 2:
                continue
            try:
                t = float(row[time_idx])
                v = float(row[torque_idx])
            except ValueError:
                continue
            data.append((t, v))

        if not data:
            return []

        t0 = data[0][0]
        values = [v for _, v in data]
        max_v = max(values) if values else 0.0
        scale = 0.1 if max_v > 100.0 else 1.0

        normalized = []
        for t, v in data:
            torque = v * scale
            if torque < 0:
                torque = 0
            if torque > 100:
                torque = 100
            normalized.append((t - t0, torque))

        return normalized

    def start_auto(self):
        if self.auto_running:
            return
        if not self.auto_paused:
            self.auto_data = self.load_auto_csv(self.auto_path.get())
            if not self.auto_data:
                self.auto_max_time = "00:00"
                self.auto_time.set("00:00/00:00")
                return
            self.auto_index = 0
            self.auto_elapsed_offset = 0.0
        elif not self.auto_data:
            self.auto_data = self.load_auto_csv(self.auto_path.get())
            if not self.auto_data:
                self.auto_max_time = "00:00"
                self.auto_time.set("00:00/00:00")
                return

        if self.auto_data:
            self.auto_max_time = self._format_time(self.auto_data[-1][0])

        self.auto_running = True
        self.auto_paused = False
        self.auto_start_time = time.monotonic() - self.auto_elapsed_offset
        if self.auto_index < len(self.auto_data):
            self.auto_torque.set(self.auto_data[self.auto_index][1])
        self.update_display()
        self._update_auto_start_label()
        self.on_auto_mode_change()

    def on_auto_mode_change(self, _event=None):
        filename = self.auto_mode_files.get(self.auto_mode.get())
        if filename:
            self.auto_path.set(filename)
        self._schedule_auto_step()

    def stop_auto(self):
        if self.auto_after_id is not None:
            self.root.after_cancel(self.auto_after_id)
            self.auto_after_id = None
        if self.auto_running and self.auto_start_time is not None:
            self.auto_elapsed_offset = time.monotonic() - self.auto_start_time
        self.auto_running = False
        self.auto_paused = True
        self.auto_start_time = None
        self._update_auto_start_label()

    def _schedule_auto_step(self):
        if not self.auto_running:
            return

        elapsed = time.monotonic() - self.auto_start_time
        current_time = self._format_time(elapsed)
        self.auto_time.set(f"{current_time}/{self.auto_max_time}")

        while self.auto_index < len(self.auto_data) and self.auto_data[self.auto_index][0] <= elapsed:
            torque = self.auto_data[self.auto_index][1]
            self.auto_torque.set(torque)
            self.auto_index += 1

        self.update_display()

        if self.auto_index >= len(self.auto_data):
            self.reset_auto()
            return

        next_t = self.auto_data[self.auto_index][0]
        delay_ms = max(1, int((next_t - elapsed) * 1000))
        self.auto_after_id = self.root.after(delay_ms, self._schedule_auto_step)

    def reset_auto(self):
        if self.auto_after_id is not None:
            self.root.after_cancel(self.auto_after_id)
            self.auto_after_id = None
        self.auto_running = False
        self.auto_paused = False
        self.auto_index = 0
        self.auto_elapsed_offset = 0.0
        self.auto_start_time = None
        self.auto_torque.set(0.0)
        self.auto_max_time = "00:00"
        self.auto_time.set("00:00/00:00")
        self.update_display()
        self._update_auto_start_label()

    def _format_time(self, seconds):
        mins = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{mins:02d}:{secs:02d}"

    def _update_auto_start_label(self):
        if hasattr(self, "auto_start_button") and self.auto_start_button is not None:
            label = "Resume" if self.auto_paused else "Start"
            self.auto_start_button.configure(text=label)

    def set_input_voltage(self, volts):
        """Update the input voltage display. Expects a number (float or int).
        """
        try:
            v = float(volts)
            # Match temperature formatting: one decimal place
            self.input_voltage.set(f"{v:.0f}")
            # Ensure UI refresh
            try:
                self.root.update_idletasks()
            except Exception:
                pass
        except (ValueError, TypeError):
            self.input_voltage.set("--")

    def set_vcu_state(self, state):
        """Set VCU state display from an integer code.

        Mapping follows the provided enum (0..8).
        """
        try:
            s = int(state)
        except (ValueError, TypeError):
            self.vcu_state.set("--")
            return

        mapping = {
            0: "   Initializing",
            1: "       Shutdown",
            2: "        Standby",
            3: "      Precharge",
            4: "Waiting for R2D",
            5: "Waiting for R2D (Auto)",
            6: "  Ready 2 Drive",
            7: "Ready (Autonomous)",
            8: "Emergency",
        }

        text = mapping.get(s)
        display = text if text is not None else "--"

        # Update UI safely from any thread
        if self.root:
            try:
                self.root.after(0, lambda: self.vcu_state.set(display))
            except Exception:
                self.vcu_state.set(display)
        else:
            self.vcu_state.set(display)

    def update_display(self):
        """Update display and publish to ROS2"""
        if self.left_rpm_valid:
            self.left_rpm_display.set(f"{self.left_motor_rpm.get():5d}")
        else:
            self.left_rpm_display.set("--")
        self._update_bar_masks(self.rpm_canvases)
        self._update_bar_masks(self.auto_bar_canvases)
        if ROS2_AVAILABLE and self.node:
            self.pub_left_torque.publish(Float32(data=self.left_torque.get()))

    def _update_bar_masks(self, bar_list):
        canvas_width, canvas_height = self.rpm_canvas_size
        for canvas, mask_id, value_var, max_value in bar_list:
            value = int(value_var.get())
            if value < 0:
                value = 0
            if value > max_value:
                value = max_value
            empty_height = int(canvas_height * (1 - (value / max_value if max_value else 0.0)))
            canvas.coords(mask_id, 0, 0, canvas_width, empty_height)

    def toggle_switch(self, var):
        var.set(not var.get())
        self.update_switch_styles()

    def update_switch_styles(self):
        self.shutdown_status.configure(
            text="ON" if self.shutdown_switch.get() else "OFF",
            foreground="green" if self.shutdown_switch.get() else "red",
        )
        self.ignition_status.configure(
            text="ON" if self.ignition_switch.get() else "OFF",
            foreground="green" if self.ignition_switch.get() else "red",
        )
        self.r2d_status.configure(
            text="ON" if self.r2d_switch.get() else "OFF",
            foreground="green" if self.r2d_switch.get() else "red",
        )

        all_on = (
            self.shutdown_switch.get()
            and self.ignition_switch.get()
            and self.r2d_switch.get()
        )
        if hasattr(self, "auto_start_button") and self.auto_start_button is not None:
            self.auto_start_button.configure(state="normal" if all_on else "disabled")

    def send_switch_status(self):
        if self.bus is None:
            return

        data = bytearray(8)
        data[0] = 1 if self.ignition_switch.get() else 0
        data[1] = 1 if self.r2d_switch.get() else 0
        data[2] = 1 if self.shutdown_switch.get() else 0

        msg = can.Message(
            arbitration_id=self.SWITCH_STATUS_CAN_ID,
            data=bytes(data),
            is_extended_id=False,
        )
        try:
            self.bus.send(msg)
        except Exception as e:
            print(f"Error sending switch status: {e}")

        # Send pump speed on CAN ID 0x751
        pump_data = bytearray(8)
        pump_data[0] = int(self.horizontal_torque.get())
        pump_msg = can.Message(
            arbitration_id=self.PUMP_SPEED_CAN_ID,
            data=bytes(pump_data),
            is_extended_id=False,
        )
        try:
            self.bus.send(pump_msg)
        except Exception as e:
            print(f"Error sending pump speed: {e}")

    def schedule_switch_send(self):
        self.send_switch_status()
        self.switch_after_id = self.root.after(
            self.switch_send_interval_ms, self.schedule_switch_send
        )

    def reset_sliders(self):
        """Reset slider to zero"""
        self.left_torque.set(0.0)
        self.horizontal_torque.set(0.0)
        self.shutdown_switch.set(False)
        self.ignition_switch.set(False)
        self.r2d_switch.set(False)
        self.update_switch_styles()
        self.update_display()

    def send_can_message(self, torque, motor_id):
        if self.bus is None:
            print("CAN bus not initialized")
            self.can_status_label.config(text="Disconnected", foreground="red")
            return

        # Pack torque into 2 bytes (signed 16-bit, big-endian)
        # NOTE: Sends value in byte1 instead of byte0.
        data = struct.pack(">h", int(torque*10))
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            print(f"Sent torque {torque}% to motor {hex(motor_id)}")
            self.can_status_label.config(text="Connected", foreground="green")
        except Exception as e:
            print(f"Error sending CAN message: {e}")
            self.can_status_label.config(text="Error", foreground="red")

    def node_id_to_can_id(self, node_id, packet_id):
        """Convert Node ID and Packet ID to CAN ID
        CAN ID = (Packet ID << 5) | Node ID
        """
        return (packet_id << 5) | node_id

    def send_torques(self):
        """Se   nd current slider value via CAN"""
        if not (self.shutdown_switch.get() and self.ignition_switch.get() and self.r2d_switch.get()):
            return
        current_tab = self.accel_notebook.index("current")
        auto_tab_index = self.accel_notebook.index(self.auto_tab)

        if current_tab == auto_tab_index:
            if not self.auto_running:
                return
            torque_value = self.auto_torque.get()
        else:
            torque_value = self.left_torque.get()

        # Get Node ID and calculate CAN ID for Set Current (0x05)
        left_node = int(self.left_node_id.get())

        left_can_id = self.node_id_to_can_id(left_node, self.SET_CURRENT_PACKET_ID)

        print(f"Left: Node ID {left_node} -> CAN ID {hex(left_can_id)}")

        self.send_can_message(torque_value, left_can_id)

    def schedule_torque_send(self):
        self.send_torques()
        self.torque_after_id = self.root.after(
            self.torque_send_interval_ms, self.schedule_torque_send
        )

    def on_closing(self):
        if self.torque_after_id is not None:
            self.root.after_cancel(self.torque_after_id)
        if self.switch_after_id is not None:
            self.root.after_cancel(self.switch_after_id)
        if self.auto_after_id is not None:
            self.root.after_cancel(self.auto_after_id)
        self.receiving = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1)
        if self.bus:
            self.bus.shutdown()
        if ROS2_AVAILABLE:
            rclpy.shutdown()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = TorqueVectoringUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
