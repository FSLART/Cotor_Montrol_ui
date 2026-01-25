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
        self.root.geometry("1200x700")
        self.root.configure(bg="#f0f0f0")

        # Style
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 12), background="#f0f0f0")
        style.configure("TButton", font=("Arial", 10))
        style.configure("TFrame", background="#f0f0f0")
        style.configure("Green.Vertical.TProgressbar", background="green")

        # CAN bus initialization
        try:
            self.bus = can.interface.Bus(channel="vcan0", interface="socketcan")
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
            self.pub_right_torque = self.node.create_publisher(
                Float32, "right_motor_torque", 10
            )
        else:
            self.node = None

        # Variables - direct motor control (0-100%)
        self.left_torque = tk.DoubleVar(value=0.0)
        self.right_torque = tk.DoubleVar(value=0.0)

        # Node ID variables (CAN ID will be calculated: CAN_ID = (Packet_ID << 5) | Node_ID)
        # Packet ID for Set Current is 0x05
        # Packet ID for General Data 3 (temperatures) is 0x22
        self.SET_CURRENT_PACKET_ID = 0x05
        self.GENERAL_DATA_3_PACKET_ID = 0x22
        self.left_node_id = tk.StringVar(value="4")
        self.right_node_id = tk.StringVar(value="25")

        # Temperature variables
        self.left_controller_temp = tk.StringVar(value="--")
        self.left_motor_temp = tk.StringVar(value="--")
        self.right_controller_temp = tk.StringVar(value="--")
        self.right_motor_temp = tk.StringVar(value="--")

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

        can_id_frame = ttk.Frame(root)
        can_id_frame.pack(pady=10, padx=10, fill="x")

        button_frame = ttk.Frame(root)
        button_frame.pack(pady=10, padx=10, fill="x")

        status_frame = ttk.Frame(root)
        status_frame.pack(pady=10, padx=10, fill="x")

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

        # Left Motor Temperatures
        temp_left_frame = ttk.Frame(left_frame)
        temp_left_frame.pack(pady=10)
        ttk.Label(temp_left_frame, text="Inverter:", font=("Arial", 10)).grid(row=0, column=0, sticky="e")
        ttk.Label(temp_left_frame, textvariable=self.left_controller_temp, font=("Arial", 10, "bold"), foreground="red").grid(row=0, column=1, sticky="w")
        ttk.Label(temp_left_frame, text="°C", font=("Arial", 10)).grid(row=0, column=2, sticky="w")
        ttk.Label(temp_left_frame, text="Motor:", font=("Arial", 10)).grid(row=1, column=0, sticky="e")
        ttk.Label(temp_left_frame, textvariable=self.left_motor_temp, font=("Arial", 10, "bold"), foreground="orange").grid(row=1, column=1, sticky="w")
        ttk.Label(temp_left_frame, text="°C", font=("Arial", 10)).grid(row=1, column=2, sticky="w")

        # Right Motor Slider
        right_frame = ttk.Frame(sliders_frame)
        right_frame.pack(side="right", expand=True, padx=30)
        
        ttk.Label(right_frame, text="Right Motor", font=("Arial", 14, "bold")).pack(pady=5)
        
        self.right_slider = tk.Scale(
            right_frame,
            from_=100,
            to=0,
            resolution=0.1,
            variable=self.right_torque,
            orient=tk.VERTICAL,
            length=200,
            width=40,
            font=("Arial", 12),
            command=lambda _: self.update_display(),
        )
        self.right_slider.pack(pady=10)
        
        ttk.Label(right_frame, textvariable=self.right_torque, font=("Arial", 16, "bold"), foreground="blue").pack()
        ttk.Label(right_frame, text="%", font=("Arial", 12)).pack()

        # Right Motor Temperatures
        temp_right_frame = ttk.Frame(right_frame)
        temp_right_frame.pack(pady=10)
        ttk.Label(temp_right_frame, text="Inverter:", font=("Arial", 10)).grid(row=0, column=0, sticky="e")
        ttk.Label(temp_right_frame, textvariable=self.right_controller_temp, font=("Arial", 10, "bold"), foreground="red").grid(row=0, column=1, sticky="w")
        ttk.Label(temp_right_frame, text="°C", font=("Arial", 10)).grid(row=0, column=2, sticky="w")
        ttk.Label(temp_right_frame, text="Motor:", font=("Arial", 10)).grid(row=1, column=0, sticky="e")
        ttk.Label(temp_right_frame, textvariable=self.right_motor_temp, font=("Arial", 10, "bold"), foreground="orange").grid(row=1, column=1, sticky="w")
        ttk.Label(temp_right_frame, text="°C", font=("Arial", 10)).grid(row=1, column=2, sticky="w")

        # Node ID Configuration (CAN ID = (Packet_ID << 5) | Node_ID)
        ttk.Label(can_id_frame, text="Left Motor Node ID:", font=("Arial", 12)).grid(
            row=0, column=0, padx=10, pady=5, sticky="w"
        )
        ttk.Entry(
            can_id_frame, textvariable=self.left_node_id, font=("Arial", 12), width=10
        ).grid(row=0, column=1, padx=10, pady=5, sticky="w")

        ttk.Label(can_id_frame, text="Right Motor Node ID:", font=("Arial", 12)).grid(
            row=0, column=2, padx=10, pady=5, sticky="w"
        )
        ttk.Entry(
            can_id_frame, textvariable=self.right_node_id, font=("Arial", 12), width=10
        ).grid(row=0, column=3, padx=10, pady=5, sticky="w")

        # Button Elements
        ttk.Button(
            button_frame, text="Send Once", command=self.send_torques
        ).grid(row=0, column=0, padx=10, pady=10)

        # Continuous send buttons
        ttk.Button(
            button_frame, text="Start Continuous Send", command=self.start_continuous
        ).grid(row=0, column=1, padx=10, pady=10)
        ttk.Button(
            button_frame, text="Stop Continuous Send", command=self.stop_continuous
        ).grid(row=0, column=2, padx=10, pady=10)
        ttk.Button(
            button_frame, text="Reset to Zero", command=self.reset_sliders
        ).grid(row=0, column=3, padx=10, pady=10)

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

        # Start continuous thread
        self.running = False
        self.thread = None

        # Start CAN receive thread for temperatures
        self.receiving = True
        self.receive_thread = threading.Thread(target=self.can_receive_loop, daemon=True)
        self.receive_thread.start()

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
            # Bytes 0-1: Controller temperature (signed 16-bit, little-endian, scale 10)
            # Bytes 2-3: Motor temperature (signed 16-bit, little-endian, scale 10)
            controller_temp_raw = struct.unpack("<h", msg.data[0:2])[0]
            motor_temp_raw = struct.unpack("<h", msg.data[2:4])[0]
            
            controller_temp = controller_temp_raw / 10.0
            motor_temp = motor_temp_raw / 10.0
            
            # Check which motor this belongs to
            try:
                left_node = int(self.left_node_id.get())
                right_node = int(self.right_node_id.get())
                
                if node_id == left_node:
                    self.left_controller_temp.set(f"{controller_temp:.1f}")
                    self.left_motor_temp.set(f"{motor_temp:.1f}")
                elif node_id == right_node:
                    self.right_controller_temp.set(f"{controller_temp:.1f}")
                    self.right_motor_temp.set(f"{motor_temp:.1f}")
            except ValueError:
                pass

    def update_display(self):
        """Update display and publish to ROS2"""
        if ROS2_AVAILABLE and self.node:
            self.pub_left_torque.publish(Float32(data=self.left_torque.get()))
            self.pub_right_torque.publish(Float32(data=self.right_torque.get()))

    def reset_sliders(self):
        """Reset both sliders to zero"""
        self.left_torque.set(0.0)
        self.right_torque.set(0.0)
        self.update_display()

    def send_can_message(self, torque, motor_id):
        if self.bus is None:
            print("CAN bus not initialized")
            self.can_status_label.config(text="Disconnected", foreground="red")
            return

        # Pack torque into 2 bytes (signed 16-bit, little-endian)
        # NOTE: If your motor expects a different scaling, adjust conversion/scaling here.
        data = struct.pack("<h", int(torque * 10))
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
        """Send current slider values via CAN"""
        # Get Node IDs and calculate CAN IDs for Set Current (0x05)
        left_node = int(self.left_node_id.get())
        right_node = int(self.right_node_id.get())
        
        left_can_id = self.node_id_to_can_id(left_node, self.SET_CURRENT_PACKET_ID)
        right_can_id = self.node_id_to_can_id(right_node, self.SET_CURRENT_PACKET_ID)
        
        print(f"Left: Node ID {left_node} -> CAN ID {hex(left_can_id)}")
        print(f"Right: Node ID {right_node} -> CAN ID {hex(right_can_id)}")
        
        self.send_can_message(self.left_torque.get(), left_can_id)
        self.send_can_message(self.right_torque.get(), right_can_id)

    def continuous_send_loop(self):
        while self.running:
            self.send_torques()
            time.sleep(0.1)  # 10 Hz

    def start_continuous(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.continuous_send_loop)
            self.thread.start()

    def stop_continuous(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def on_closing(self):
        self.stop_continuous()
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
