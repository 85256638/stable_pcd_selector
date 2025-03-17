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

###############################################################################
# CRC16 and Packet Building Functions (Livox-specific)
###############################################################################
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

###############################################################################
# Main GUI Class
###############################################################################
class ROSLauncherGUI:
    def __init__(self, root):
        self.root = root

        # ---------------------------
        # Define Essential Attributes First
        # ---------------------------
        self.process_livox = None
        self.process_camera = None
        self.process_detector = None
        self.process_lidar = None
        self.running = {"livox": False, "camera": False, "detector": False, "lidar": False}
        self.lidar_ips = {
            "farleft_2": "192.168.1.2",
            "left_57":   "192.168.1.57",
            "mid_65":    "192.168.1.65",
            "right_75":  "192.168.1.75"
        }
        self.lidar_port = 8001
        self.ping_labels = {}

        # ---------------------------
        # Set up Translations for Languages
        # ---------------------------
        self.translations = {
            "English": {
                "title": "Chengdu TAIXING Refractory Wear Measurement System",
                "tab1": "Launch",
                "tab2": "Control",
                "tab3": "Data",
                "main_drive_usage": "Main Drive Usage",
                "refresh_usage": "Refresh Usage",
                "launch_ros_livox": "Launch ROS Livox Driver",
                "terminate_ros_livox": "Terminate ROS Livox Driver",
                "launch_ros_camera": "Launch ROS Camera Driver",
                "terminate_ros_camera": "Terminate ROS Camera Driver",
                "launch_person_detector": "Launch Person Detector",
                "terminate_person_detector": "Terminate Person Detector",
                "launch_lidar_trigger": "Launch LiDAR Trigger Node",
                "terminate_lidar_trigger": "Terminate LiDAR Trigger Node",
                "launch_all": "Launch All",
                "terminate_all": "Terminate All",
                "quit": "Quit",
                "lidar_ip_addresses": "LiDAR IP Addresses (LiDAR IPs are configured by Livox Viewer only)",
                "save_ip_addresses": "Save IP Addresses",
                "lidar_normal_mode": "Set LiDAR to Normal Mode",
                "lidar_power_mode": "Set LiDAR to Power Saving Mode",
                "lidar_status": "LiDAR Status",
                "rostopic_frame": "ROStopics (starting with '/livox/')",
                "refresh_topics": "Refresh Topics",
                "saved_locations": "Saved Locations",
                "pointcloud_dir": "Point clouds saved location:",
                "camera_dir": "Camera images saved location:",
                "update_saved_locations": "Update Saved Locations",
                "disk_usage_text": "Used: {used_gb:.1f} GB ({percent_used:.1f}%) / Total: {total_gb:.1f} GB, Free: {free_gb:.1f} GB"
            },
            "Chinese": {
                "title": "成都钛兴炉衬残厚测量系统",
                "tab1": "启动",
                "tab2": "控制",
                "tab3": "数据",
                "main_drive_usage": "主硬盘使用情况",
                "refresh_usage": "刷新",
                "launch_ros_livox": "启动 LiDAR驱动",
                "terminate_ros_livox": "终止 LiDAR驱动",
                "launch_ros_camera": "启动 相机驱动",
                "terminate_ros_camera": "终止 相机驱动",
                "launch_person_detector": "启动 人员检测",
                "terminate_person_detector": "终止 人员检测",
                "launch_lidar_trigger": "启动 LiDAR触发节点",
                "terminate_lidar_trigger": "终止 LiDAR触发节点",
                "launch_all": "全部启动",
                "terminate_all": "全部终止",
                "quit": "退出",
                "lidar_ip_addresses": "LiDAR IP地址（由Livox Viewer配置）",
                "save_ip_addresses": "保存IP地址",
                "lidar_normal_mode": "设置LiDAR为正常模式",
                "lidar_power_mode": "设置LiDAR为省电模式",
                "lidar_status": "LiDAR状态",
                "rostopic_frame": "ROS 话题（以'/livox/'开头）",
                "refresh_topics": "刷新话题",
                "saved_locations": "保存位置",
                "pointcloud_dir": "点云保存位置：",
                "camera_dir": "相机图像保存位置：",
                "update_saved_locations": "更新保存位置",
                "disk_usage_text": "已用: {used_gb:.1f} GB ({percent_used:.1f}%) / 总: {total_gb:.1f} GB, 可用: {free_gb:.1f} GB"
            }
        }
        self.current_language = "English"

        # ---------------------------
        # LiDAR Name Translations (for Tab 2 labels)
        # ---------------------------
        self.lidar_name_translation = {
            "farleft_2": {"English": "farleft_2", "Chinese": "最左2"},
            "left_57":   {"English": "left_57",   "Chinese": "左57"},
            "mid_65":    {"English": "mid_65",    "Chinese": "中65"},
            "right_75":  {"English": "right_75",  "Chinese": "右75"}
        }

        # ---------------------------
        # Create a Top Bar for Language Selection
        # ---------------------------
        top_bar = tk.Frame(root)
        top_bar.pack(side="top", fill="x")
        self.language_var = tk.StringVar(value="English")
        self.language_menu = tk.OptionMenu(top_bar, self.language_var, "English", "中文", command=self.on_language_change)
        self.language_menu.pack(side="right", padx=10, pady=5)

        # ---------------------------
        # Create Notebook and Tabs
        # ---------------------------
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)

        # Tab 1: Launch
        self.tab1 = ttk.Frame(self.notebook)
        self.notebook.add(self.tab1, text=self.translations[self.current_language]["tab1"])
        self.tab1.columnconfigure(0, weight=1)
        self.tab1.columnconfigure(1, weight=1)

        # Disk usage bar in Tab 1
        self.storage_frame = tk.LabelFrame(self.tab1, text=self.translations[self.current_language]["main_drive_usage"], padx=5, pady=5)
        self.storage_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.disk_usage_canvas = tk.Canvas(self.storage_frame, height=50)
        self.disk_usage_canvas.pack(fill="both", expand=True, padx=5, pady=5)
        self.refresh_usage_button = tk.Button(self.storage_frame, text=self.translations[self.current_language]["refresh_usage"], command=self.update_storage_bar)
        self.refresh_usage_button.pack(pady=2)
        # Update immediately after layout is complete
        self.root.after_idle(self.update_storage_bar)

        # Launch/Terminate Buttons in Tab 1
        self.button_livox = tk.Button(self.tab1, width=30, height=2, command=self.launch_livox_driver)
        self.button_livox.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.terminate_livox_button = tk.Button(self.tab1, width=30, height=2, command=self.terminate_livox_driver)
        self.terminate_livox_button.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        self.button_camera = tk.Button(self.tab1, width=30, height=2, command=self.launch_camera_driver)
        self.button_camera.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.terminate_camera_button = tk.Button(self.tab1, width=30, height=2, command=self.terminate_camera_driver)
        self.terminate_camera_button.grid(row=2, column=1, padx=5, pady=5, sticky="ew")

        self.button_detector = tk.Button(self.tab1, width=30, height=2, command=self.launch_person_detector)
        self.button_detector.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        self.terminate_detector_button = tk.Button(self.tab1, width=30, height=2, command=self.terminate_person_detector)
        self.terminate_detector_button.grid(row=3, column=1, padx=5, pady=5, sticky="ew")

        self.button_lidar = tk.Button(self.tab1, width=30, height=2, command=self.launch_lidar_trigger)
        self.button_lidar.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        self.terminate_lidar_button = tk.Button(self.tab1, width=30, height=2, command=self.terminate_lidar_trigger)
        self.terminate_lidar_button.grid(row=4, column=1, padx=5, pady=5, sticky="ew")

        self.launch_all_button = tk.Button(self.tab1, width=30, height=2, command=self.launch_all)
        self.launch_all_button.grid(row=5, column=0, padx=5, pady=10, sticky="ew")
        self.terminate_all_button = tk.Button(self.tab1, width=30, height=2, command=self.terminate_all)
        self.terminate_all_button.grid(row=5, column=1, padx=5, pady=10, sticky="ew")

        # Tab 2: Control
        self.tab2 = ttk.Frame(self.notebook)
        self.notebook.add(self.tab2, text=self.translations[self.current_language]["tab2"])
        self.tab2.columnconfigure(0, weight=1)
        self.tab2.columnconfigure(1, weight=1)

        self.ip_frame = tk.LabelFrame(self.tab2, text=self.translations[self.current_language]["lidar_ip_addresses"], padx=5, pady=5)
        self.ip_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.lidar_ip_entries = {}
        self.ping_labels = {}
        self.lidar_ip_order = ["farleft_2", "left_57", "mid_65", "right_75"]
        self.lidar_ip_labels = {}
        row_index = 0
        for idx, key in enumerate(self.lidar_ip_order, start=1):
            name = self.lidar_name_translation[key][self.current_language]
            label_text = f"LiDAR {idx}: {name}"
            lbl = tk.Label(self.ip_frame, text=label_text, anchor="w", width=25)
            lbl.grid(row=row_index, column=0, padx=5, pady=2, sticky="w")
            self.lidar_ip_labels[key] = lbl
            entry = tk.Entry(self.ip_frame, width=15)
            entry.insert(0, self.lidar_ips[key])
            entry.grid(row=row_index, column=1, padx=5, pady=2, sticky="ew")
            btn = tk.Button(self.ip_frame, text="Ping", command=lambda k=key: self.ping_ip(k), width=5)
            btn.grid(row=row_index, column=2, padx=5, pady=2)
            ping_lbl = tk.Label(self.ip_frame, text="", width=15)
            ping_lbl.grid(row=row_index, column=3, padx=5, pady=2)
            self.lidar_ip_entries[key] = entry
            self.ping_labels[key] = ping_lbl
            row_index += 1

        self.save_ip_button = tk.Button(self.ip_frame, text=self.translations[self.current_language]["save_ip_addresses"], command=self.save_ip_addresses)
        self.save_ip_button.grid(row=row_index, column=0, columnspan=4, pady=5)

        self.lidar_normal_button = tk.Button(self.tab2, width=30, height=2, command=self.set_normal_mode)
        self.lidar_normal_button.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.lidar_power_button = tk.Button(self.tab2, width=30, height=2, command=self.set_power_saving_mode)
        self.lidar_power_button.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        self.status_frame = tk.LabelFrame(self.tab2, text=self.translations[self.current_language]["lidar_status"], padx=5, pady=5)
        self.status_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.lidar_status_labels = {}
        for key, ip_str in self.lidar_ips.items():
            lbl = tk.Label(self.status_frame, text=f"{key} ({ip_str}): Unknown")
            lbl.pack(anchor="w", padx=5, pady=2)
            self.lidar_status_labels[key] = lbl

        self.rostopic_frame = tk.LabelFrame(self.tab2, text=self.translations[self.current_language]["rostopic_frame"], padx=5, pady=5)
        self.rostopic_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.rostopic_error_label = tk.Label(self.rostopic_frame, text="", fg="red")
        self.rostopic_error_label.pack(anchor="n", pady=2)
        self.rostopic_listbox = tk.Listbox(self.rostopic_frame, width=50, height=8)
        self.rostopic_listbox.pack(side="left", fill="both", expand=True, padx=5, pady=2)
        scrollbar = tk.Scrollbar(self.rostopic_frame, orient="vertical")
        scrollbar.pack(side="right", fill="y")
        self.rostopic_listbox.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.rostopic_listbox.yview)
        self.refresh_topics_button = tk.Button(self.rostopic_frame, text=self.translations[self.current_language]["refresh_topics"], command=self.update_rostopic_list)
        self.refresh_topics_button.pack(side="bottom", pady=5)

        # Tab 3: Data
        self.tab3 = ttk.Frame(self.notebook)
        self.notebook.add(self.tab3, text=self.translations[self.current_language]["tab3"])
        self.tab3.columnconfigure(0, weight=1)

        self.loc_frame = tk.LabelFrame(self.tab3, text=self.translations[self.current_language]["saved_locations"], padx=5, pady=5)
        self.loc_frame.pack(fill="x", padx=5, pady=5)
        self.loc_pointcloud_label = tk.Label(self.loc_frame, text=self.translations[self.current_language]["pointcloud_dir"])
        self.loc_pointcloud_label.grid(row=0, column=0, sticky="e", padx=5, pady=2)
        self.pointclouds_location = tk.Entry(self.loc_frame, width=50)
        self.pointclouds_location.insert(0, "/home/taixing/pointclouds/")
        self.pointclouds_location.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        self.pointclouds_browse_button = tk.Button(self.loc_frame, text="Browse", command=self.browse_pointclouds_location)
        self.pointclouds_browse_button.grid(row=0, column=2, padx=5, pady=2)
        self.loc_camera_label = tk.Label(self.loc_frame, text=self.translations[self.current_language]["camera_dir"])
        self.loc_camera_label.grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.camera_images_location = tk.Entry(self.loc_frame, width=50)
        self.camera_images_location.insert(0, "/home/taixing/Cam-2-Trigger-Lidar/processed_images")
        self.camera_images_location.grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        self.camera_browse_button = tk.Button(self.loc_frame, text="Browse", command=self.browse_camera_images_location)
        self.camera_browse_button.grid(row=1, column=2, padx=5, pady=2)
        self.update_locations_button = tk.Button(self.loc_frame, text=self.translations[self.current_language]["update_saved_locations"], command=self.update_saved_locations)
        self.update_locations_button.grid(row=2, column=0, columnspan=3, pady=5)

        # ---------------------------
        # Create a Bottom Frame for the Quit Button
        # ---------------------------
        bottom_bar = tk.Frame(root)
        bottom_bar.pack(side="bottom", fill="x")
        self.quit_button = tk.Button(bottom_bar, width=10, height=1, command=self.root.quit)
        self.quit_button.pack(side="right", padx=10, pady=10)
        # Set the Quit button text according to language
        self.quit_button.config(text=self.translations[self.current_language]["quit"])

        # ---------------------------
        # Start periodic updates
        # ---------------------------
        self.update_lidar_status()
        self.update_rostopic_list()
        self.update_launch_button_states()

        # Set initial language texts
        self.set_language(self.current_language)

    ########################################################################
    # Language Methods
    ########################################################################
    def on_language_change(self, selected_lang):
        lang = "Chinese" if selected_lang == "中文" else selected_lang
        self.set_language(lang)

    def set_language(self, lang):
        self.current_language = lang
        tr = self.translations[lang]

        # Update window title
        self.root.title(tr["title"])

        # Update tab titles
        self.notebook.tab(0, text=tr["tab1"])
        self.notebook.tab(1, text=tr["tab2"])
        self.notebook.tab(2, text=tr["tab3"])

        # Tab 1: Launch
        self.storage_frame.config(text=tr["main_drive_usage"])
        self.refresh_usage_button.config(text=tr["refresh_usage"])
        self.button_livox.config(text=tr["launch_ros_livox"])
        self.terminate_livox_button.config(text=tr["terminate_ros_livox"])
        self.button_camera.config(text=tr["launch_ros_camera"])
        self.terminate_camera_button.config(text=tr["terminate_ros_camera"])
        self.button_detector.config(text=tr["launch_person_detector"])
        self.terminate_detector_button.config(text=tr["terminate_person_detector"])
        self.button_lidar.config(text=tr["launch_lidar_trigger"])
        self.terminate_lidar_button.config(text=tr["terminate_lidar_trigger"])
        self.launch_all_button.config(text=tr["launch_all"])
        self.terminate_all_button.config(text=tr["terminate_all"])

        # Tab 2: Control
        self.ip_frame.config(text=tr["lidar_ip_addresses"])
        self.save_ip_button.config(text=tr["save_ip_addresses"])
        self.lidar_normal_button.config(text=tr["lidar_normal_mode"])
        self.lidar_power_button.config(text=tr["lidar_power_mode"])
        self.status_frame.config(text=tr["lidar_status"])
        self.rostopic_frame.config(text=tr["rostopic_frame"])
        self.refresh_topics_button.config(text=tr["refresh_topics"])
        for idx, key in enumerate(self.lidar_ip_order, start=1):
            name = self.lidar_name_translation[key][self.current_language]
            label_text = f"LiDAR {idx}: {name}"
            if key in self.lidar_ip_labels:
                self.lidar_ip_labels[key].config(text=label_text)

        # Tab 3: Data
        self.loc_frame.config(text=tr["saved_locations"])
        self.loc_pointcloud_label.config(text=tr["pointcloud_dir"])
        self.loc_camera_label.config(text=tr["camera_dir"])
        self.update_locations_button.config(text=tr["update_saved_locations"])

        # Update Quit button text
        self.quit_button.config(text=tr["quit"])

    ########################################################################
    # Tab 1: Launch Button State Updater
    ########################################################################
    def update_launch_button_states(self):
        self.button_livox.config(state="disabled" if self.running["livox"] else "normal")
        self.button_camera.config(state="disabled" if self.running["camera"] else "normal")
        self.button_detector.config(state="disabled" if self.running["detector"] else "normal")
        self.button_lidar.config(state="disabled" if self.running["lidar"] else "normal")
        self.root.after(1000, self.update_launch_button_states)

    ########################################################################
    # Tab 1: Disk Usage Bar
    ########################################################################
    def update_storage_bar(self):
        total, used, free = shutil.disk_usage("/")
        percent_used = (used / total) * 100.0
        canvas_width = self.disk_usage_canvas.winfo_width()
        if canvas_width < 10:
            canvas_width = 400
        canvas_height = 50
        used_width = int(canvas_width * (used / total))
        self.disk_usage_canvas.delete("all")
        self.disk_usage_canvas.create_rectangle(0, 0, used_width, canvas_height, fill="red")
        self.disk_usage_canvas.create_rectangle(used_width, 0, canvas_width, canvas_height, fill="green")
        total_gb = total / (1024**3)
        used_gb = used / (1024**3)
        free_gb = free / (1024**3)
        tr = self.translations[self.current_language]
        text_str = tr["disk_usage_text"].format(used_gb=used_gb, total_gb=total_gb, free_gb=free_gb, percent_used=percent_used)
        self.disk_usage_canvas.create_text(canvas_width/2, canvas_height/2, text=text_str, fill="white")
        self.root.after(5000, self.update_storage_bar)

    ########################################################################
    # Tab 3: Data - Saved Locations
    ########################################################################
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

    ########################################################################
    # Tab 2: LiDAR IP & Ping Methods
    ########################################################################
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

    ########################################################################
    # Terminal Launching Helper
    ########################################################################
    def launch_in_terminal(self, command, env=None):
        return subprocess.Popen(
            ["gnome-terminal", "--disable-factory", "--", "bash", "-c", command + "; exec bash"],
            preexec_fn=os.setsid,
            env=env
        )

    ########################################################################
    # ROS Nodes Launch/Terminate Methods
    ########################################################################
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

    ########################################################################
    # LiDAR Mode Commands (Livox SDK protocol)
    ########################################################################
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

    ########################################################################
    # Query LiDAR Status
    ########################################################################
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
                text=f"{key} ({self.lidar_ips[key]}): {status_str}"))
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
                self.root.after(0, lambda e=e: self.rostopic_error_label.config(text=f"Error updating rostopics: {e}"))
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
