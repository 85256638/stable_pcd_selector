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
import json

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
    Structure: Header (0xAA55) + CommandID (1 byte) + Reserved (1 byte, fixed to 0x00) +
               Payload Length (2 bytes, little endian) + Payload (variable) +
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
        # Essential Attributes
        # ---------------------------
        self.process_livox = None
        self.process_camera = None
        self.process_detector = None
        self.process_lidar = None
        self.running = {"livox": False, "camera": False, "detector": False, "lidar": False}
        self.lidar_ips = {
            "farleft_2": "192.168.1.2",
            "left_57": "192.168.1.57",
            "mid_65": "192.168.1.65",
            "right_75": "192.168.1.75"
        }
        self.lidar_port = 8001
        self.ping_labels = {}
        self.lidar_status_vars = {}  # Used for LiDAR channels

        # ---------------------------
        # Translations / Parameter Labels
        # ---------------------------
        self.translations = {
            "English": {
                "title": "Chengdu TAIXING Refractory Wear Measurement System",
                "tab1": "Launch",
                "tab2": "Control",
                "tab3": "Data",
                "tab4": "Parameters",
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
                "lidar_ip_addresses": "LiDAR IP Addresses (configured by Livox Viewer)",
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
                "disk_usage_text": "Used: {used_gb:.1f} GB ({percent_used:.1f}%) / Total: {total_gb:.1f} GB, Free: {free_gb:.1f} GB",
                # LiDAR Trigger Node Parameters
                "lidar_trigger_params": "LiDAR Trigger Node Parameters",
                "frame_time": "Frame Time (sec)",
                "base_output_dir": "Base Output Directory",
                "lidar_status_label": "Active LiDARs",
                # Camera Person Detector Parameters
                "camera_detector_params": "Camera Person Detector Parameters",
                "yolo_model_path": "YOLO Model Path",
                "save_raw_images": "Save Raw Images",
                "save_processed_images": "Save Processed Images",
                "confidence_threshold": "Confidence Threshold",
                "person_class_id": "Person Class ID",
                "simulated_person_detection": "Simulated Person Detection",
                "latching_behavior": "Latching Behavior for /trigger_lidar",
                # New Simulated Detection Options & Image Session Directory
                "simulated_detection_option": "Simulated Detection Option",
                "simulated_detection_option_always": "always",
                "simulated_detection_option_interval": "interval",
                "simulated_detection_on_duration": "Simulated On Duration (mins)",
                "simulated_detection_cycle": "Detection Cycle (mins)",
                "image_session_base_dir": "Image Session Base Directory",
                "save_parameters": "Save Parameters",
                "image_save_interval": "Image Save Interval (sec)"
            },
            "Chinese": {
                "title": "成都钛兴炉衬残厚测量系统",
                "tab1": "启动",
                "tab2": "控制",
                "tab3": "数据",
                "tab4": "参数",
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
                "disk_usage_text": "已用: {used_gb:.1f} GB ({percent_used:.1f}%) / 总: {total_gb:.1f} GB, 可用: {free_gb:.1f} GB",
                # LiDAR触发节点参数
                "lidar_trigger_params": "LiDAR触发节点参数",
                "frame_time": "帧时间 (秒)",
                "base_output_dir": "点云保存目录",
                "lidar_status_label": "启用的 LiDAR",
                # 人员检测器参数
                "camera_detector_params": "人员检测器参数",
                "yolo_model_path": "YOLO 模型路径",
                "save_raw_images": "保存原始图像",
                "save_processed_images": "保存处理后图像",
                "confidence_threshold": "置信度阈值",
                "person_class_id": "人员类别 ID",
                "simulated_person_detection": "模拟人员检测",
                "latching_behavior": "/trigger_lidar 锁存行为",
                # 新的模拟检测选项及图像会话目录
                "simulated_detection_option": "模拟检测选项",
                "simulated_detection_option_always": "always",
                "simulated_detection_option_interval": "interval",
                "simulated_detection_on_duration": "开启时长 (分钟)",
                "simulated_detection_cycle": "检测周期 (分钟)",
                "image_session_base_dir": "图像会话基本目录",
                "save_parameters": "保存参数",
                "image_save_interval": "图像保存间隔 (秒)"
            }
        }
        self.current_language = "English"

        # ---------------------------
        # LiDAR Name Translations
        # ---------------------------
        self.lidar_name_translation = {
            "farleft_2": {"English": "farleft_2", "Chinese": "最左2"},
            "left_57": {"English": "left_57", "Chinese": "左57"},
            "mid_65": {"English": "mid_65", "Chinese": "中65"},
            "right_75": {"English": "right_75", "Chinese": "右75"}
        }

        # ---------------------------
        # Top Bar: Language Selection
        # ---------------------------
        top_bar = tk.Frame(root)
        top_bar.pack(side="top", fill="x")
        self.language_var = tk.StringVar(value="English")
        self.language_menu = tk.OptionMenu(top_bar, self.language_var, "English", "中文", command=self.on_language_change)
        self.language_menu.pack(side="right", padx=10, pady=5)

        # ---------------------------
        # Notebook and Tabs
        # ---------------------------
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)

        # Tab 1: Launch
        self.tab1 = ttk.Frame(self.notebook)
        self.notebook.add(self.tab1, text=self.translations[self.current_language]["tab1"])
        self.tab1.columnconfigure(0, weight=1)
        self.tab1.columnconfigure(1, weight=1)
        self.storage_frame = tk.LabelFrame(self.tab1, text=self.translations[self.current_language]["main_drive_usage"], padx=5, pady=5)
        self.storage_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
        self.disk_usage_canvas = tk.Canvas(self.storage_frame, height=50)
        self.disk_usage_canvas.pack(fill="both", expand=True, padx=5, pady=5)
        self.refresh_usage_button = tk.Button(self.storage_frame, text=self.translations[self.current_language]["refresh_usage"], command=self.update_storage_bar)
        self.refresh_usage_button.pack(pady=2)
        self.root.after_idle(self.update_storage_bar)
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
        self.pointclouds_location.insert(0, "/home/taixing/Cam-2-Trigger-Lidar_DATASAVES/pointclouds/")
        self.pointclouds_location.grid(row=0, column=1, padx=5, pady=2, sticky="ew")
        self.pointclouds_browse_button = tk.Button(self.loc_frame, text="Browse", command=self.browse_pointclouds_location)
        self.pointclouds_browse_button.grid(row=0, column=2, padx=5, pady=2)
        self.loc_camera_label = tk.Label(self.loc_frame, text=self.translations[self.current_language]["camera_dir"])
        self.loc_camera_label.grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.camera_images_location = tk.Entry(self.loc_frame, width=50)
        self.camera_images_location.insert(0, "/home/taixing/Cam-2-Trigger-Lidar_DATASAVES/images")
        self.camera_images_location.grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        self.camera_browse_button = tk.Button(self.loc_frame, text="Browse", command=self.browse_camera_images_location)
        self.camera_browse_button.grid(row=1, column=2, padx=5, pady=2)
        self.update_locations_button = tk.Button(self.loc_frame, text=self.translations[self.current_language]["update_saved_locations"], command=self.update_saved_locations)
        self.update_locations_button.grid(row=2, column=0, columnspan=3, pady=5)

        # Tab 4: Parameters
        self.tab4 = ttk.Frame(self.notebook)
        self.notebook.add(self.tab4, text=self.translations[self.current_language]["tab4"])
        self.tab4.columnconfigure(0, weight=1)
        self.build_tab4_content()

        # Bottom Bar: Quit
        bottom_bar = tk.Frame(self.root)
        bottom_bar.pack(side="bottom", fill="x")
        self.quit_button = tk.Button(bottom_bar, width=10, height=1, command=self.root.quit)
        self.quit_button.pack(side="right", padx=10, pady=10)
        self.quit_button.config(text=self.translations[self.current_language]["quit"])

        # Start periodic updates
        self.update_lidar_status()
        self.update_rostopic_list()
        self.update_launch_button_states()
        self.set_language(self.current_language)

    def build_tab4_content(self):
        tr = self.translations[self.current_language]
        try:
            current_frame_time = self.frame_time_entry.get()
        except Exception:
            current_frame_time = "1.0"
        try:
            current_base_output = self.base_output_dir_entry.get()
        except Exception:
            current_base_output = "/home/taixing/Cam-2-Trigger-Lidar_DATASAVES/pointclouds/"

        # LiDAR Trigger Node Parameters
        lidar_param_frame = tk.LabelFrame(self.tab4, text=tr["lidar_trigger_params"], padx=5, pady=5)
        lidar_param_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        tk.Label(lidar_param_frame, text=tr["frame_time"]).grid(row=0, column=0, sticky="e", padx=5, pady=2)
        self.frame_time_entry = tk.Entry(lidar_param_frame, width=20)
        self.frame_time_entry.insert(0, current_frame_time)
        self.frame_time_entry.grid(row=0, column=1, padx=5, pady=2)
        tk.Label(lidar_param_frame, text=tr["base_output_dir"]).grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.base_output_dir_entry = tk.Entry(lidar_param_frame, width=40)
        self.base_output_dir_entry.insert(0, current_base_output)
        self.base_output_dir_entry.grid(row=1, column=1, padx=5, pady=2)
        tk.Label(lidar_param_frame, text=tr["lidar_status_label"]).grid(row=2, column=0, sticky="ne", padx=5, pady=2)
        status_frame = tk.Frame(lidar_param_frame)
        status_frame.grid(row=2, column=1, sticky="w", padx=5, pady=2)
        for key in ["farleft_2", "left_57", "mid_65", "right_75"]:
            var = self.lidar_status_vars.get(key, tk.BooleanVar(value=True))
            self.lidar_status_vars[key] = var
            cb = tk.Checkbutton(status_frame, text=key, variable=var)
            cb.pack(side="left", padx=2)

        # Camera Person Detector Parameters
        camera_param_frame = tk.LabelFrame(self.tab4, text=tr["camera_detector_params"], padx=5, pady=5)
        camera_param_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        tk.Label(camera_param_frame, text=tr["yolo_model_path"]).grid(row=0, column=0, sticky="e", padx=5, pady=2)
        self.yolo_model_path_entry = tk.Entry(camera_param_frame, width=40)
        self.yolo_model_path_entry.insert(0, os.environ.get("YOLO_MODEL_PATH", "/home/taixing/yolov8n.onnx"))
        self.yolo_model_path_entry.grid(row=0, column=1, padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["save_raw_images"]).grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.save_raw_images_var = tk.BooleanVar(value=os.environ.get("SAVE_RAW_IMAGES", "True").lower() == "true")
        self.save_raw_images_cb = tk.Checkbutton(camera_param_frame, variable=self.save_raw_images_var)
        self.save_raw_images_cb.grid(row=1, column=1, sticky="w", padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["save_processed_images"]).grid(row=2, column=0, sticky="e", padx=5, pady=2)
        self.save_processed_images_var = tk.BooleanVar(value=os.environ.get("SAVE_PROCESSED_IMAGES", "False").lower() == "true")
        self.save_processed_images_cb = tk.Checkbutton(camera_param_frame, variable=self.save_processed_images_var)
        self.save_processed_images_cb.grid(row=2, column=1, sticky="w", padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["confidence_threshold"]).grid(row=3, column=0, sticky="e", padx=5, pady=2)
        self.confidence_threshold_entry = tk.Entry(camera_param_frame, width=20)
        self.confidence_threshold_entry.insert(0, os.environ.get("CONFIDENCE_THRESHOLD", "0.25"))
        self.confidence_threshold_entry.grid(row=3, column=1, padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["person_class_id"]).grid(row=4, column=0, sticky="e", padx=5, pady=2)
        self.person_class_id_entry = tk.Entry(camera_param_frame, width=20)
        self.person_class_id_entry.insert(0, os.environ.get("PERSON_CLASS_ID", "0"))
        self.person_class_id_entry.grid(row=4, column=1, padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["simulated_person_detection"]).grid(row=5, column=0, sticky="e", padx=5, pady=2)
        self.simulated_person_detection_var = tk.BooleanVar(value=os.environ.get("SIMULATED_PERSON_DETECTION", "True").lower() == "true")
        self.simulated_person_detection_cb = tk.Checkbutton(camera_param_frame, variable=self.simulated_person_detection_var)
        self.simulated_person_detection_cb.grid(row=5, column=1, sticky="w", padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["latching_behavior"]).grid(row=6, column=0, sticky="e", padx=5, pady=2)
        self.latching_behavior_var = tk.BooleanVar(value=os.environ.get("LATCHING_BEHAVIOR", "True").lower() == "true")
        self.latching_behavior_cb = tk.Checkbutton(camera_param_frame, variable=self.latching_behavior_var)
        self.latching_behavior_cb.grid(row=6, column=1, sticky="w", padx=5, pady=2)
        tk.Label(camera_param_frame, text=tr["image_save_interval"]).grid(row=7, column=0, sticky="e", padx=5, pady=2)
        self.image_save_interval_entry = tk.Entry(camera_param_frame, width=20)
        self.image_save_interval_entry.insert(0, os.environ.get("IMAGE_SAVE_INTERVAL", "1"))
        self.image_save_interval_entry.grid(row=7, column=1, padx=5, pady=2)

        # Simulated Detection Options & Image Session Base Directory
        sim_det_frame = tk.LabelFrame(self.tab4, text=tr["simulated_detection_option"], padx=5, pady=5)
        sim_det_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        tk.Label(sim_det_frame, text=tr["simulated_detection_option"]).grid(row=0, column=0, sticky="e", padx=5, pady=2)
        self.simulated_detection_option_var = tk.StringVar(value=os.environ.get("SIMULATED_DETECTION_OPTION", "always"))
        self.simulated_detection_option_menu = ttk.Combobox(sim_det_frame, textvariable=self.simulated_detection_option_var, state="readonly",
                                                            values=[tr["simulated_detection_option_always"], tr["simulated_detection_option_interval"]])
        self.simulated_detection_option_menu.grid(row=0, column=1, padx=5, pady=2)
        tk.Label(sim_det_frame, text=tr["simulated_detection_on_duration"]).grid(row=1, column=0, sticky="e", padx=5, pady=2)
        self.simulated_detection_on_duration_entry = tk.Entry(sim_det_frame, width=10)
        self.simulated_detection_on_duration_entry.insert(0, os.environ.get("SIMULATED_DETECTION_ON_DURATION", "0.1"))
        self.simulated_detection_on_duration_entry.grid(row=1, column=1, padx=5, pady=2)
        tk.Label(sim_det_frame, text=tr["simulated_detection_cycle"]).grid(row=2, column=0, sticky="e", padx=5, pady=2)
        self.simulated_detection_cycle_entry = tk.Entry(sim_det_frame, width=10)
        self.simulated_detection_cycle_entry.insert(0, os.environ.get("SIMULATED_DETECTION_CYCLE", "1"))
        self.simulated_detection_cycle_entry.grid(row=2, column=1, padx=5, pady=2)
        tk.Label(sim_det_frame, text=tr["image_session_base_dir"]).grid(row=3, column=0, sticky="e", padx=5, pady=2)
        self.image_session_base_dir_entry = tk.Entry(sim_det_frame, width=40)
        self.image_session_base_dir_entry.insert(0, os.environ.get("IMAGE_SESSION_BASE_DIR", "/home/taixing/Cam-2-Trigger-Lidar_DATASAVES/images"))
        self.image_session_base_dir_entry.grid(row=3, column=1, padx=5, pady=2)

        # Save Parameters Button
        self.save_parameters_button = tk.Button(self.tab4, text=tr["save_parameters"], command=self.save_parameters)
        self.save_parameters_button.grid(row=3, column=0, padx=5, pady=10)

    def save_parameters(self):
        # LiDAR Trigger Node parameters
        frame_time = self.frame_time_entry.get().strip()
        base_output_dir = self.base_output_dir_entry.get().strip()
        lidar_status = {key: var.get() for key, var in self.lidar_status_vars.items()}
        # Camera Person Detector parameters
        yolo_model_path = self.yolo_model_path_entry.get().strip()
        save_raw_images = self.save_raw_images_var.get()
        save_processed_images = self.save_processed_images_var.get()
        confidence_threshold = self.confidence_threshold_entry.get().strip()
        person_class_id = self.person_class_id_entry.get().strip()
        simulated_person_detection = self.simulated_person_detection_var.get()
        latching_behavior = self.latching_behavior_var.get()
        # Simulated Detection Options and Image Session Base Directory
        simulated_detection_option = self.simulated_detection_option_var.get()
        simulated_detection_on_duration = self.simulated_detection_on_duration_entry.get().strip()
        simulated_detection_cycle = self.simulated_detection_cycle_entry.get().strip()
        image_session_base_dir = self.image_session_base_dir_entry.get().strip()
        image_save_interval = self.image_save_interval_entry.get().strip()
        print("  Image Save Interval:", image_save_interval)
        os.environ["IMAGE_SAVE_INTERVAL"] = image_save_interval

        print("Saved Parameters:")
        print("LiDAR Trigger Node:")
        print("  Frame Time:", frame_time)
        print("  Base Output Directory:", base_output_dir)
        print("  LiDAR Status:", lidar_status)
        print("Camera Person Detector:")
        print("  YOLO Model Path:", yolo_model_path)
        print("  Save Raw Images:", save_raw_images)
        print("  Save Processed Images:", save_processed_images)
        print("  Confidence Threshold:", confidence_threshold)
        print("  Person Class ID:", person_class_id)
        print("  Simulated Person Detection:", simulated_person_detection)
        print("  Latching Behavior for /trigger_lidar:", latching_behavior)
        print("Simulated Detection Options:")
        print("  Option:", simulated_detection_option)
        print("  On Duration (mins):", simulated_detection_on_duration)
        print("  Detection Cycle (mins):", simulated_detection_cycle)
        print("Image Session Base Directory:", image_session_base_dir)
        # Update environment variables for the launched nodes.
        os.environ["FRAME_TIME"] = frame_time
        os.environ["BASE_OUTPUT_DIR"] = base_output_dir
        os.environ["LIDAR_STATUS"] = json.dumps(lidar_status)
        os.environ["YOLO_MODEL_PATH"] = yolo_model_path
        os.environ["SAVE_RAW_IMAGES"] = str(save_raw_images)
        os.environ["SAVE_PROCESSED_IMAGES"] = str(save_processed_images)
        os.environ["CONFIDENCE_THRESHOLD"] = confidence_threshold
        os.environ["PERSON_CLASS_ID"] = person_class_id
        os.environ["SIMULATED_PERSON_DETECTION"] = str(simulated_person_detection)
        os.environ["LATCHING_BEHAVIOR"] = str(latching_behavior)
        os.environ["SIMULATED_DETECTION_OPTION"] = simulated_detection_option
        os.environ["SIMULATED_DETECTION_ON_DURATION"] = simulated_detection_on_duration
        os.environ["SIMULATED_DETECTION_CYCLE"] = simulated_detection_cycle
        os.environ["IMAGE_SESSION_BASE_DIR"] = image_session_base_dir
        print("Environment variables updated.")

    def on_language_change(self, selected_lang):
        lang = "Chinese" if selected_lang == "中文" else selected_lang
        self.set_language(lang)

    def set_language(self, lang):
        self.current_language = lang
        tr = self.translations[lang]
        self.root.title(tr["title"])
        self.notebook.tab(0, text=tr["tab1"])
        self.notebook.tab(1, text=tr["tab2"])
        self.notebook.tab(2, text=tr["tab3"])
        self.notebook.tab(3, text=tr["tab4"])
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
        self.loc_frame.config(text=tr["saved_locations"])
        self.loc_pointcloud_label.config(text=tr["pointcloud_dir"])
        self.loc_camera_label.config(text=tr["camera_dir"])
        self.update_locations_button.config(text=tr["update_saved_locations"])
        for widget in self.tab4.winfo_children():
            widget.destroy()
        self.build_tab4_content()
        self.quit_button.config(text=tr["quit"])

    def update_launch_button_states(self):
        self.button_livox.config(state="disabled" if self.running["livox"] else "normal")
        self.button_camera.config(state="disabled" if self.running["camera"] else "normal")
        self.button_detector.config(state="disabled" if self.running["detector"] else "normal")
        self.button_lidar.config(state="disabled" if self.running["lidar"] else "normal")
        self.root.after(1000, self.update_launch_button_states)

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
        text_str = self.translations[self.current_language]["disk_usage_text"].format(used_gb=used_gb, total_gb=total_gb, free_gb=free_gb, percent_used=percent_used)
        self.disk_usage_canvas.create_text(canvas_width/2, canvas_height/2, text=text_str, fill="white")
        self.root.after(5000, self.update_storage_bar)

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
                output = subprocess.check_output(["env", "LC_ALL=C", "ping", "-c", "1", ip], universal_newlines=True)
                for line in output.splitlines():
                    if "time=" in line:
                        idx = line.find("time=")
                        time_str = line[idx:].split()[0].split("=")[1]
                        latency = float(time_str)
                        self.root.after(0, lambda: self.ping_labels[lidar_key].config(text=f"{latency:.1f} ms", fg="green" if latency <= 1000 else "red"))
                        return
                self.root.after(0, lambda: self.ping_labels[lidar_key].config(text="unreachable", fg="red"))
            except subprocess.CalledProcessError:
                self.root.after(0, lambda: self.ping_labels[lidar_key].config(text="unreachable", fg="red"))
        threading.Thread(target=run_ping, daemon=True).start()

    def launch_in_terminal(self, command, env=None):
        return subprocess.Popen(["gnome-terminal", "--disable-factory", "--", "bash", "-c", command + "; exec bash"],
                                preexec_fn=os.setsid, env=env)

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
            # Explicitly export IMAGE_SAVE_INTERVAL so the updated value is used.
            cmd = "export IMAGE_SAVE_INTERVAL={}; python3 '/home/taixing/Cam-2-Trigger-Lidar/multiple lidars/camera_person_detector.py'".format(os.environ["IMAGE_SAVE_INTERVAL"])
            full_cmd = cmd + "; exec bash"
            self.process_detector = subprocess.Popen(
                ["gnome-terminal", "--disable-factory", "--", "bash", "--noprofile", "--norc", "-c", full_cmd],
                preexec_fn=os.setsid,
                env=env
            )
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
            cmd = "python3 '/home/taixing/Cam-2-Trigger-Lidar/multiple lidars/lidar_trigger_node_multi.py'"
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
            self.root.after(0, lambda: self.lidar_status_labels[key].config(text=f"{key} ({self.lidar_ips[key]}): {status_str}"))
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
