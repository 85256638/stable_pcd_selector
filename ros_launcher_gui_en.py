#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, filedialog
import subprocess
import time
import threading
import os
import signal
import socket
import shutil

# Use TkAgg backend for matplotlib
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def crc16(data: bytes) -> int:
    """Compute CRC16 (CCITT) using polynomial 0x1021 with initial value 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021)
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def build_packet(command_id: int, payload: bytes) -> bytes:
    """
    Build a Livox command packet.
    
    Structure:
      Header (0xAA55) +
      CommandID (1 byte) +
      Reserved (1 byte, fixed to 0x00) +
      Payload Length (2 bytes, little endian) +
      Payload (variable) +
      CRC16 (2 bytes, little endian).
    """
    header = b'\xAA\x55'
    reserved = b'\x00'
    payload_length = len(payload).to_bytes(2, 'little')
    data_for_crc = bytes([command_id]) + reserved + payload_length + payload
    crc = crc16(data_for_crc)
    crc_bytes = crc.to_bytes(2, 'little')
    return header + data_for_crc + crc_bytes

class ROSLauncherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Chengdu TAIXING Refractory Wear Measurement System")

        # Node processes & running flags
        self.process_livox = None
        self.process_camera = None
        self.process_detector = None
        self.process_lidar = None
        self.running = {"livox": False, "camera": False, "detector": False, "lidar": False}

        # LiDAR IP dictionary
        self.lidar_ips = {
            "farleft_2": "192.168.1.2",
            "left_57":   "192.168.1.57",
            "mid_65":    "192.168.1.65",
            "right_75":  "192.168.1.75"
        }
        self.lidar_port = 8001
        self.ping_labels = {}

        # Create Notebook with three tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)

        # =========================
        # Tab 1: "Launch"
        # =========================
        tab1 = ttk.Frame(self.notebook)
        self.notebook.add(tab1, text="Launch")
        tab1.columnconfigure(0, weight=1)
        tab1.columnconfigure(1, weight=1)

        # Disk usage bar at the top
        storage_frame = tk.LabelFrame(tab1, text="Main Drive Usage", padx=5, pady=5)
        storage_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.disk_usage_canvas = tk.Canvas(storage_frame, height=50)
        self.disk_usage_canvas.pack(fill="both", expand=True, padx=5, pady=5)
        tk.Button(storage_frame, text="Refresh Usage", command=self.update_storage_bar).pack(pady=2)

        # Launch/Terminate Buttons
        self.button_livox = tk.Button(tab1, text="Launch ROS Livox Driver",
                                      command=self.launch_livox_driver, width=30, height=2)
        self.button_livox.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(tab1, text="Terminate ROS Livox Driver", command=self.terminate_livox_driver,
                  width=30, height=2).grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        self.button_camera = tk.Button(tab1, text="Launch ROS Camera Driver",
                                       command=self.launch_camera_driver, width=30, height=2)
        self.button_camera.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(tab1, text="Terminate ROS Camera Driver", command=self.terminate_camera_driver,
                  width=30, height=2).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

        self.button_detector = tk.Button(tab1, text="Launch Person Detector",
                                         command=self.launch_person_detector, width=30, height=2)
        self.button_detector.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(tab1, text="Terminate Person Detector", command=self.terminate_person_detector,
                  width=30, height=2).grid(row=3, column=1, padx=5, pady=5, sticky="ew")

        self.button_lidar = tk.Button(tab1, text="Launch LiDAR Trigger Node",
                                      command=self.launch_lidar_trigger, width=30, height=2)
        self.button_lidar.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(tab1, text="Terminate LiDAR Trigger Node", command=self.terminate_lidar_trigger,
                  width=30, height=2).grid(row=4, column=1, padx=5, pady=5, sticky="ew")

        tk.Button(tab1, text="Launch All", command=self.launch_all, width=30, height=2)\
            .grid(row=5, column=0, padx=5, pady=10, sticky="ew")
        tk.Button(tab1, text="Terminate All", command=self.terminate_all, width=30, height=2)\
            .grid(row=5, column=1, padx=5, pady=10, sticky="ew")
        tk.Button(tab1, text="Quit", command=root.quit, width=65, height=2)\
            .grid(row=6, column=0, columnspan=2, padx=5, pady=10)

        # We will update the disk usage bar once the GUI is laid out
        self.root.after(200, self.update_storage_bar)

        self.update_launch_button_states()

        # =========================
        # Tab 2: "Control"
        # =========================
        tab2 = ttk.Frame(self.notebook)
        self.notebook.add(tab2, text="Control")
        tab2.columnconfigure(0, weight=1)
        tab2.columnconfigure(1, weight=1)

        # LabelFrame with updated text
        ip_frame = tk.LabelFrame(tab2, text="LiDAR IP Addresses (LiDAR IPs are configured by Livox Viewer only)",
                                 padx=5, pady=5)
        ip_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.lidar_ip_entries = {}

        # The order for labeling LiDAR 1..4
        order = ["farleft_2", "left_57", "mid_65", "right_75"]
        row_index = 0
        for idx, key in enumerate(order, start=1):
            # Remove "IP" from the text
            label_text = f"LiDAR {idx}: {key}"
            lbl = tk.Label(ip_frame, text=label_text, anchor="w", width=25)
            lbl.grid(row=row_index, column=0, padx=5, pady=2, sticky="w")

            entry = tk.Entry(ip_frame, width=15)
            entry.insert(0, self.lidar_ips[key])
            entry.grid(row=row_index, column=1, padx=5, pady=2, sticky="ew")

            btn = tk.Button(ip_frame, text="Ping", command=lambda k=key: self.ping_ip(k), width=5)
            btn.grid(row=row_index, column=2, padx=5, pady=2)

            ping_lbl = tk.Label(ip_frame, text="", width=15)
            ping_lbl.grid(row=row_index, column=3, padx=5, pady=2)

            self.lidar_ip_entries[key] = entry
            self.ping_labels[key] = ping_lbl

            row_index += 1

        tk.Button(ip_frame, text="Save IP Addresses", command=self.save_ip_addresses)\
            .grid(row=row_index, column=0, columnspan=4, pady=5)

        tk.Button(tab2, text="Set LiDAR to Normal Mode", command=self.set_normal_mode, width=30, height=2)\
            .grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        tk.Button(tab2, text="Set LiDAR to Power Saving Mode", command=self.set_power_saving_mode, width=30, height=2)\
            .grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        self.status_frame = tk.LabelFrame(tab2, text="LiDAR Status", padx=5, pady=5)
        self.status_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.lidar_status_labels = {}
        for key, ip_str in self.lidar_ips.items():
            lbl = tk.Label(self.status_frame, text=f"{key} ({ip_str}): Unknown")
            lbl.pack(anchor="w", padx=5, pady=2)
            self.lidar_status_labels[key] = lbl

        rostopic_frame = tk.LabelFrame(tab2, text="ROStopics (starting with '/livox/')", padx=5, pady=5)
        rostopic_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.rostopic_error_label = tk.Label(rostopic_frame, text="", fg="red")
        self.rostopic_error_label.pack(anchor="n", pady=2)
        self.rostopic_listbox = tk.Listbox(rostopic_frame, width=50, height=8)
        self.rostopic_listbox.pack(side="left", fill="both", expand=True, padx=5, pady=2)
        scrollbar = tk.Scrollbar(rostopic_frame, orient="vertical")
        scrollbar.pack(side="right", fill="y")
        self.rostopic_listbox.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.rostopic_listbox.yview)
        tk.Button(rostopic_frame, text="Refresh Topics", command=self.update_rostopic_list)\
            .pack(side="bottom", pady=5)

        # =========================
        # Tab 3: "Data"
        # =========================
        tab3 = ttk.Frame(self.notebook)
        self.notebook.add(tab3, text="Data")
        tab3.columnconfigure(0, weight=1)

        loc_frame = tk.LabelFrame(tab3, text="Saved Locations", padx=5, pady=5)
        loc_frame.pack(fill="x", padx=5, pady=5)

        tk.Label(loc_frame, text="Point clouds saved location:").grid(row=0, column=0, sticky="e", padx=5, pady=2)
        self.pointclouds_location = tk.Entry(loc_frame, width=50)
        self.pointclouds_location.insert(0, "/home/taixing/pointclouds/")
        self.pointclouds_location.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        tk.Button(loc_frame, text="Browse", command=self.browse_pointclouds_location)\
            .grid(row=0, column=2, padx=5, pady=2)

        tk.Label(loc_frame, text="Camera images saved location:").grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.camera_images_location = tk.Entry(loc_frame, width=50)
        self.camera_images_location.insert(0, "/home/taixing/Cam-2-Trigger-Lidar/processed_images")
        self.camera_images_location.grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        tk.Button(loc_frame, text="Browse", command=self.browse_camera_images_location)\
            .grid(row=1, column=2, padx=5, pady=2)

        tk.Button(loc_frame, text="Update Saved Locations", command=self.update_saved_locations)\
            .grid(row=2, column=0, columnspan=3, pady=5)

        # Periodic updates for LiDAR status and rostopic list
        self.update_lidar_status()
        self.update_rostopic_list()
        # Periodically update launch button states
        self.update_launch_button_states()

    # --------------------------
    # Tab 1: Launch Button State Updater
    # --------------------------
    def update_launch_button_states(self):
        self.button_livox.config(state="disabled" if self.running["livox"] else "normal")
        self.button_camera.config(state="disabled" if self.running["camera"] else "normal")
        self.button_detector.config(state="disabled" if self.running["detector"] else "normal")
        self.button_lidar.config(state="disabled" if self.running["lidar"] else "normal")
        self.root.after(1000, self.update_launch_button_states)

    # --------------------------
    # Disk Usage Bar (Tab 1)
    # --------------------------
    def update_storage_bar(self):
        total, used, free = shutil.disk_usage("/")
        percent_used = used / total
        canvas_width = self.disk_usage_canvas.winfo_width()
        if canvas_width < 10:
            canvas_width = 400
        canvas_height = 50
        used_width = int(canvas_width * percent_used)
        self.disk_usage_canvas.delete("all")
        self.disk_usage_canvas.create_rectangle(0, 0, used_width, canvas_height, fill="red")
        self.disk_usage_canvas.create_rectangle(used_width, 0, canvas_width, canvas_height, fill="green")
        total_gb = total / (1024**3)
        used_gb = used / (1024**3)
        free_gb = free / (1024**3)
        text_str = f"Used: {used_gb:.1f} GB ({percent_used*100:.1f}%) / Total: {total_gb:.1f} GB, Free: {free_gb:.1f} GB"
        self.disk_usage_canvas.create_text(canvas_width/2, canvas_height/2, text=text_str, fill="white")
        # Refresh again every 5 seconds
        self.root.after(5000, self.update_storage_bar)

    # --------------------------
    # Tab 3: Data - Saved Locations
    # --------------------------
    def update_saved_locations(self):
        pointclouds_dir = self.pointclouds_location.get().strip()
        camera_images_dir = self.camera_images_location.get().strip()
        os.environ["BASE_OUTPUT_DIR"] = pointclouds_dir
        os.environ["PROCESSED_IMAGE_DIR"] = camera_images_dir
        print("Updated Saved Locations:")
        print("BASE_OUTPUT_DIR =", pointclouds_dir)
        print("PROCESSED_IMAGE_DIR =", camera_images_dir)

    def browse_pointclouds_location(self):
        directory = filedialog.askdirectory()
        if directory:
            self.pointclouds_location.delete(0, tk.END)
            self.pointclouds_location.insert(0, directory)

    def browse_camera_images_location(self):
        directory = filedialog.askdirectory()
        if directory:
            self.camera_images_location.delete(0, tk.END)
            self.camera_images_location.insert(0, directory)

    # --------------------------
    # Tab 2: LiDAR IP & Ping
    # --------------------------
    def save_ip_addresses(self):
        for key, entry in self.lidar_ip_entries.items():
            new_ip = entry.get().strip()
            self.lidar_ips[key] = new_ip
            self.lidar_status_labels[key].config(text=f"{key} ({new_ip}): Unknown")
        print("Updated LiDAR IP addresses:", self.lidar_ips)

    def ping_ip(self, lidar_key):
        ip = self.lidar_ips[lidar_key]
        def run_ping():
            try:
                output = subprocess.check_output(
                    ["env", "LC_ALL=C", "ping", "-c", "1", ip],
                    universal_newlines=True
                )
                for line in output.splitlines():
                    if "time=" in line:
                        idx = line.find("time=")
                        time_str = line[idx:].split()[0].split("=")[1]
                        latency = float(time_str)
                        self.root.after(0, lambda: self.ping_labels[lidar_key].config(
                            text=f"{latency:.1f} ms", fg="green" if latency <= 1000 else "red"))
                        return
                self.root.after(0, lambda: self.ping_labels[lidar_key].config(text="unreachable", fg="red"))
            except subprocess.CalledProcessError:
                self.root.after(0, lambda: self.ping_labels[lidar_key].config(text="unreachable", fg="red"))
        threading.Thread(target=run_ping, daemon=True).start()

    # --------------------------
    # Terminal launching helper
    # --------------------------
    def launch_in_terminal(self, command, env=None):
        return subprocess.Popen(
            ["gnome-terminal", "--disable-factory", "--", "bash", "-c", command + "; exec bash"],
            preexec_fn=os.setsid,
            env=env
        )

    # --------------------------
    # ROS Nodes Launch/Terminate
    # --------------------------
    def launch_livox_driver(self):
        if not self.running["livox"]:
            cmd = "roslaunch livox_ros_driver livox_lidar_multi.launch"
            self.process_livox = self.launch_in_terminal(cmd)
            self.running["livox"] = True
            print("Launched ROS Livox Driver")
        else:
            print("ROS Livox Driver is already running")

    def terminate_livox_driver(self):
        if self.running["livox"]:
            try:
                os.killpg(os.getpgid(self.process_livox.pid), signal.SIGTERM)
                print("Terminated ROS Livox Driver")
            except Exception as e:
                print("Error terminating ROS Livox Driver:", e)
            self.running["livox"] = False
            self.process_livox = None
        else:
            print("ROS Livox Driver is not running")

    def launch_camera_driver(self):
        if not self.running["camera"]:
            cmd = "roslaunch hik_camera_driver camera.launch"
            self.process_camera = self.launch_in_terminal(cmd)
            self.running["camera"] = True
            print("Launched ROS Camera Driver")
        else:
            print("ROS Camera Driver is already running")

    def terminate_camera_driver(self):
        if self.running["camera"]:
            try:
                os.killpg(os.getpgid(self.process_camera.pid), signal.SIGTERM)
                print("Terminated ROS Camera Driver")
            except Exception as e:
                print("Error terminating ROS Camera Driver:", e)
            self.running["camera"] = False
            self.process_camera = None
        else:
            print("ROS Camera Driver is not running")

    def launch_person_detector(self):
        if not self.running["detector"]:
            env = os.environ.copy()
            cmd = "python3 '/home/taixing/Cam-2-Trigger-Lidar/multiple lidars/camera_person_detector.py'"
            self.process_detector = self.launch_in_terminal(cmd, env=env)
            self.running["detector"] = True
            print("Launched Person Detector Node")
        else:
            print("Person Detector Node is already running")

    def terminate_person_detector(self):
        if self.running["detector"]:
            try:
                os.killpg(os.getpgid(self.process_detector.pid), signal.SIGTERM)
                print("Terminated Person Detector Node")
            except Exception as e:
                print("Error terminating Person Detector Node:", e)
            self.running["detector"] = False
            self.process_detector = None
        else:
            print("Person Detector Node is not running")

    def launch_lidar_trigger(self):
        if not self.running["lidar"]:
            env = os.environ.copy()
            cmd = "python3 '/home/taixing/Cam-2-Trigger-Lidar/multiple lidars/lidar_trigger_node.py'"
            self.process_lidar = self.launch_in_terminal(cmd, env=env)
            self.running["lidar"] = True
            print("Launched LiDAR Trigger Node")
        else:
            print("LiDAR Trigger Node is already running")

    def terminate_lidar_trigger(self):
        if self.running["lidar"]:
            try:
                os.killpg(os.getpgid(self.process_lidar.pid), signal.SIGTERM)
                print("Terminated LiDAR Trigger Node")
            except Exception as e:
                print("Error terminating LiDAR Trigger Node:", e)
            self.running["lidar"] = False
            self.process_lidar = None
        else:
            print("LiDAR Trigger Node is not running")

    def launch_all(self):
        def sequence():
            self.launch_livox_driver()
            time.sleep(5)
            self.launch_camera_driver()
            time.sleep(5)
            self.launch_person_detector()
            time.sleep(5)
            self.launch_lidar_trigger()
        threading.Thread(target=sequence, daemon=True).start()

    def terminate_all(self):
        self.terminate_livox_driver()
        self.terminate_camera_driver()
        self.terminate_person_detector()
        self.terminate_lidar_trigger()

    # --------------------------
    # LiDAR Mode Commands (Livox SDK)
    # --------------------------
    def set_normal_mode(self):
        self.send_mode_command(0)

    def set_power_saving_mode(self):
        self.send_mode_command(1)

    def send_mode_command(self, mode):
        packet = build_packet(0x03, bytes([mode]))
        print("Sending mode command packet:", packet.hex())
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for ip_str in self.lidar_ips.values():
                sock.sendto(packet, (ip_str, self.lidar_port))
                print(f"Sent mode command to {ip_str}:{self.lidar_port} with mode {mode}")
        except Exception as e:
            print("Failed to send mode command:", e)
        finally:
            sock.close()

    # --------------------------
    # Query LiDAR Status
    # --------------------------
    def query_lidar_status(self, ip_str):
        packet = build_packet(0x06, b'')
        print(f"Querying LiDAR at {ip_str} with packet: {packet.hex()}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3.0)
        try:
            sock.sendto(packet, (ip_str, self.lidar_port))
            data, addr = sock.recvfrom(1024)
            print(f"Received raw data from {ip_str}: {data.hex()}")
            if len(data) < 6 or data[0:2] != b'\xAA\x55':
                return None
            payload_length = int.from_bytes(data[4:6], 'little')
            if len(data) < 6 + payload_length + 2:
                return None
            payload = data[6:6+payload_length]
            crc_received = int.from_bytes(data[6+payload_length:6+payload_length+2], 'little')
            data_for_crc = data[2:6+payload_length]
            if crc16(data_for_crc) != crc_received:
                print(f"CRC check failed for response from {ip_str}")
                return None
            if payload_length == 1:
                return payload[0]
            return None
        except Exception as e:
            print(f"Query failed for LiDAR at {ip_str}: {e}")
        finally:
            sock.close()
        return None

    def update_lidar_status(self):
        def query_and_update(key, ip_str):
            mode = self.query_lidar_status(ip_str)
            if mode is None:
                status_str = "Disconnected/Unknown"
            else:
                if mode == 0:
                    status_str = "Normal"
                elif mode == 1:
                    status_str = "Power Saving"
                elif mode == 2:
                    status_str = "Standby"
                else:
                    status_str = f"Unknown ({mode})"
            self.root.after(0, lambda: self.lidar_status_labels[key].config(
                text=f"{key} ({ip_str}): {status_str}"))
        for key, ip_str in self.lidar_ips.items():
            threading.Thread(target=query_and_update, args=(key, ip_str), daemon=True).start()
        self.root.after(5000, self.update_lidar_status)

    def update_rostopic_list(self):
        def query_topics():
            try:
                output = subprocess.check_output(["rostopic", "list"], universal_newlines=True)
                topics = output.splitlines()
                filtered = [t for t in topics if t.startswith("/livox/")]
                self.root.after(0, lambda: self._update_topic_listbox(filtered, ""))
            except Exception as e:
                self.root.after(0, lambda: self.rostopic_error_label.config(
                    text=f"Error updating rostopics: {e}"))
        threading.Thread(target=query_topics, daemon=True).start()
        self.root.after(5000, self.update_rostopic_list)

    def _update_topic_listbox(self, topics, error_text):
        self.rostopic_listbox.delete(0, tk.END)
        for t in topics:
            self.rostopic_listbox.insert(tk.END, t)
        self.rostopic_error_label.config(text=error_text)

if __name__ == "__main__":
    root = tk.Tk()
    app = ROSLauncherGUI(root)
    root.mainloop()
