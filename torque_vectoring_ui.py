#!/usr/bin/env python3.12
import tkinter as tk
from tkinter import ttk
import can
import struct
import threading
import time

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
        self.root.geometry("675x700")
        self.root.configure(bg="#f0f0f0")

        # Style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", font=("Arial", 12), background="#f0f0f0")
        style.configure("TButton", font=("Arial", 10))
        style.configure("TFrame", background="#f0f0f0")
        style.configure("Green.Vertical.TProgressbar", background="green")
        style.configure("SwitchStatus.TLabel", font=("Arial", 12, "bold"))

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
        self.horizontal_torque = tk.DoubleVar(value=0.0)

        # Node ID variables (CAN ID will be calculated: CAN_ID = (Packet_ID << 5) | Node_ID)
        # Packet ID for Set Current is 0x05
        # Packet ID for General Data 3 (temperatures) is 0x22
        self.SET_CURRENT_PACKET_ID = 0x05
        self.GENERAL_DATA_3_PACKET_ID = 0x22
        # Switch status CAN ID (from your screenshot)
        self.SWITCH_STATUS_CAN_ID = 0x750
        self.PUMP_SPEED_CAN_ID = 0x751
        self.left_node_id = tk.StringVar(value="4")

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

        # Left Motor Slider
        left_frame = ttk.Frame(sliders_frame)
        left_frame.pack(side="left", expand=True, padx=30)
        
        ttk.Label(left_frame, text="Left Motor", font=("Arial", 14, "bold")).pack(pady=5)
        
        self.left_slider = tk.Scale(
            left_frame,
            from_=100,
            to=0,
            resolution=0.1,
            variable=self.left_torque,
            orient=tk.VERTICAL,
            length=200,
            width=40,
            font=("Arial", 12),
            command=lambda _: self.update_display(),
        )
        self.left_slider.pack(pady=10)
        
        ttk.Label(left_frame, textvariable=self.left_torque, font=("Arial", 16, "bold"), foreground="blue").pack()
        ttk.Label(left_frame, text="%", font=("Arial", 12)).pack()

        # Center switches
        switch_frame = ttk.Frame(sliders_frame)
        switch_frame.pack(side="left", expand=True, padx=30)

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
        can_id_frame.pack(pady=10, padx=10, fill="x")
        button_frame.pack(pady=10, padx=10, fill="x")
        status_frame.pack(pady=10, padx=10, fill="x")

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
        # Extract Node ID and Packet ID from CAN ID
        node_id = msg.arbitration_id & 0x1F
        packet_id = msg.arbitration_id >> 5
        
        # Check if this is a General Data 3 message (0x22) - temperatures
        if packet_id == self.GENERAL_DATA_3_PACKET_ID and len(msg.data) >= 4:
            # Bytes 0-1: Controller temperature (signed 16-bit, big-endian, scale 10)
            # Bytes 2-3: Motor temperature (signed 16-bit, big-endian, scale 10)
            controller_temp_raw = struct.unpack(">h", msg.data[0:2])[0]
            motor_temp_raw = struct.unpack(">h", msg.data[2:4])[0]
            
            controller_temp = controller_temp_raw / 10.0
            motor_temp = motor_temp_raw / 10.0
            
            # Check which motor this belongs to
            try:
                left_node = int(self.left_node_id.get())
                
                if node_id == left_node:
                    self.left_controller_temp.set(f"{controller_temp:.1f}")
                    self.left_motor_temp.set(f"{motor_temp:.1f}")
            except ValueError:
                pass

    def update_display(self):
        """Update display and publish to ROS2"""
        if ROS2_AVAILABLE and self.node:
            self.pub_left_torque.publish(Float32(data=self.left_torque.get()))

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

        # Pack torque into 2 bytes (signed 16-bit, little-endian)
        # NOTE: If your motor expects a different scaling, adjust conversion/scaling here.
        data = struct.pack("<h", int(torque / 10))
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
        """Send current slider value via CAN"""
        if not (self.shutdown_switch.get() and self.ignition_switch.get() and self.r2d_switch.get()):
            return
        # Get Node ID and calculate CAN ID for Set Current (0x05)
        left_node = int(self.left_node_id.get())
        
        left_can_id = self.node_id_to_can_id(left_node, self.SET_CURRENT_PACKET_ID)
        
        print(f"Left: Node ID {left_node} -> CAN ID {hex(left_can_id)}")
        
        self.send_can_message(self.left_torque.get(), left_can_id)

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
