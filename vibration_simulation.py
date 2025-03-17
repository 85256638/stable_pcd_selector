#!/usr/bin/env python3
"""
IMU Vibration PCD Batch Filter and Simulation (GUI Version)

Description:
  This script performs two main functions:

  1. Batch Process:
     - The user selects a base folder.
     - The program recursively finds all IMU CSV files (filenames containing "imu").
     - For each CSV file, it groups the data by the "Timestamp of last PCD" column and 
       detects which groups (timestamps) have any IMU reading (computed as sqrt(X²+Y²+Z²))
       deviating from 1.0 g by more than the specified threshold.
     - It then forms a unified set of event timestamps from all CSV files.
     - For each sensor, the script extracts an identifier from its CSV filename (e.g.
       "imu_data_mid_65.csv" yields identifier "mid_65") and determines the sensor folder.
       It also extracts a session time range from the sensor’s parent folder (which is assumed 
       to contain a substring of the form "session_YYYY-MM-DD_HH-MM-SS_to_YYYY-MM-DD_HH-MM-SS").
     - For each unified event timestamp that falls within the session range, the script searches 
       in the sensor folder for a .pcd file whose name contains both the sensor’s identifier and a 
       transformed timestamp pattern (by replacing spaces and colons with underscores).
     - If a matching file is found, it is removed (or only logged if dry-run mode is enabled).
       In the log, if the sensor’s own CSV detected the vibration at that timestamp, the removal log 
       is highlighted in red; otherwise, it is highlighted in yellow.

  2. Simulation:
     - The user can select a single CSV file for FFT analysis and vibration simulation.
     - The simulation displays a rectangle (representing a LiDAR with real dimensions 77 mm × 84.1 mm) 
       moving vertically according to the computed displacement amplitude.
     - The coordinate axes are set to x: [-290, 290] mm and y: [-122.5, 122.5] mm.
     
Usage:
  - For batch processing, select a base folder. The script will recursively search for IMU CSV files.
  - For simulation, use the "Simulate Vibration" button to select a single CSV file.
"""

import os
import re
import math
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
import open3d as o3d

class IMUPCDFilterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Vibration PCD Batch Filter and Simulation")
        self.base_folder = ""
        self.anim = None  # Save animation object to prevent garbage collection
        self.create_widgets()

    def create_widgets(self):
        # Base Folder selection
        folder_frame = tk.LabelFrame(self.root, text="Base Folder")
        folder_frame.pack(padx=10, pady=5, fill="x")
        tk.Label(folder_frame, text="Folder:").grid(row=0, column=0, sticky="w")
        self.folder_entry = tk.Entry(folder_frame, width=50)
        self.folder_entry.grid(row=0, column=1, padx=5, pady=2)
        tk.Button(folder_frame, text="Browse...", command=self.browse_base_folder).grid(row=0, column=2, padx=5)
        
        # Parameters for processing
        param_frame = tk.LabelFrame(self.root, text="Parameters")
        param_frame.pack(padx=10, pady=5, fill="x")
        tk.Label(param_frame, text="Vibration Threshold (g):").grid(row=0, column=0, sticky="w")
        self.threshold_entry = tk.Entry(param_frame, width=10)
        self.threshold_entry.insert(0, "0.001")
        self.threshold_entry.grid(row=0, column=1, padx=5, pady=2, sticky="w")
        self.dry_run_var = tk.BooleanVar(value=True)
        tk.Checkbutton(param_frame, text="Dry Run (do not delete files)", variable=self.dry_run_var).grid(row=0, column=2, padx=5, sticky="w")
        
        # Buttons for batch processing and simulation
        button_frame = tk.Frame(self.root)
        button_frame.pack(padx=10, pady=5, fill="x")
        tk.Button(button_frame, text="Batch Process", command=self.batch_process).pack(side="left", padx=5)
        tk.Button(button_frame, text="Simulate Vibration", command=self.simulate_vibration).pack(side="left", padx=5)
        
        # Log display with tag configuration
        self.log_text = scrolledtext.ScrolledText(self.root, width=80, height=15)
        self.log_text.pack(padx=10, pady=10, fill="both", expand=True)
        self.log_text.tag_config("red", foreground="red")
        self.log_text.tag_config("yellow", foreground="goldenrod")
    
    def browse_base_folder(self):
        folder_path = filedialog.askdirectory(title="Select Base Folder")
        if folder_path:
            self.base_folder = folder_path
            self.folder_entry.delete(0, tk.END)
            self.folder_entry.insert(0, folder_path)
            
    def log(self, message, tag=None):
        if tag:
            self.log_text.insert(tk.END, message + "\n", tag)
        else:
            self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
    
    def find_csv_files(self, base_folder):
        csv_files = []
        for root_dir, dirs, files in os.walk(base_folder):
            for f in files:
                if f.lower().endswith(".csv") and "imu" in f.lower():
                    csv_files.append(os.path.join(root_dir, f))
        return csv_files
    
    def extract_identifier(self, csv_filename):
        m = re.search(r'imu_data_(.+)\.csv', os.path.basename(csv_filename), re.IGNORECASE)
        if m:
            return m.group(1)
        else:
            return os.path.splitext(os.path.basename(csv_filename))[0]
    
    def extract_session_range(self, session_folder):
        # Expected pattern: session_YYYY-MM-DD_HH-MM-SS_to_YYYY-MM-DD_HH-MM-SS
        folder_name = os.path.basename(session_folder)
        pattern = r'session_(?P<start>\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})_to_(?P<end>\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})'
        m = re.search(pattern, folder_name, re.IGNORECASE)
        if m:
            try:
                start_dt = datetime.strptime(m.group("start"), "%Y-%m-%d_%H-%M-%S")
                end_dt = datetime.strptime(m.group("end"), "%Y-%m-%d_%H-%M-%S")
                return start_dt, end_dt
            except Exception as e:
                self.log(f"Error parsing session range: {e}")
                return None, None
        else:
            return None, None

    def batch_process(self):
        if not self.base_folder:
            messagebox.showerror("Error", "Please select a Base Folder first.")
            return
        csv_files = self.find_csv_files(self.base_folder)
        if not csv_files:
            self.log("No IMU CSV files found in the selected folder.")
            return
        self.log(f"Found {len(csv_files)} IMU CSV file(s) for batch processing.")
        
        try:
            threshold = float(self.threshold_entry.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Invalid threshold value.")
            return
        
        dry_run = self.dry_run_var.get()
        unified_events = set()
        # First pass: build unified event timestamps from all CSV files.
        for csv_file in csv_files:
            self.log(f"Processing CSV file: {csv_file}")
            try:
                df = pd.read_csv(csv_file)
            except Exception as e:
                self.log(f"Failed to read {csv_file}: {e}")
                continue
            missing = [col for col in ["Timestamp of last PCD", "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"] if col not in df.columns]
            if missing:
                self.log(f"CSV {csv_file} missing required columns: {missing}")
                continue
            groups = df.groupby("Timestamp of last PCD")
            for ts, group in groups:
                deviations = group[["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]].apply(
                    lambda row: abs(math.sqrt(row["Linear Acceleration X"]**2 +
                                               row["Linear Acceleration Y"]**2 +
                                               row["Linear Acceleration Z"]**2) - 1.0),
                    axis=1
                )
                if (deviations > threshold).any():
                    unified_events.add(ts.strip())
        self.log(f"Unified event timestamps detected: {sorted(unified_events)}")
        
        total_removed = 0
        # Second pass: for each CSV file, compute local events and remove matching .pcd files.
        for csv_file in csv_files:
            sensor_identifier = self.extract_identifier(csv_file)
            sensor_folder = os.path.dirname(csv_file)
            session_folder = os.path.dirname(sensor_folder)
            session_start, session_end = self.extract_session_range(session_folder)
            if session_start is None or session_end is None:
                self.log(f"Could not extract session range from folder: {session_folder}. Skipping file {csv_file}.")
                continue
            self.log(f"Processing sensor '{sensor_identifier}' in session folder: {session_folder}")
            # Compute local events for this sensor
            try:
                df = pd.read_csv(csv_file)
            except Exception as e:
                self.log(f"Failed to re-read {csv_file}: {e}")
                continue
            local_events = set()
            groups = df.groupby("Timestamp of last PCD")
            for ts, group in groups:
                deviations = group[["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]].apply(
                    lambda row: abs(math.sqrt(row["Linear Acceleration X"]**2 +
                                               row["Linear Acceleration Y"]**2 +
                                               row["Linear Acceleration Z"]**2) - 1.0),
                    axis=1
                )
                if (deviations > threshold).any():
                    local_events.add(ts.strip())
            # For each unified event (if within session range), search for matching .pcd file.
            for event_ts in unified_events:
                try:
                    event_dt = datetime.strptime(event_ts, "%Y-%m-%d %H:%M:%S")
                except Exception as e:
                    self.log(f"Error parsing event timestamp {event_ts}: {e}")
                    continue
                if not (session_start <= event_dt <= session_end):
                    continue
                pattern = event_ts.replace(" ", "_").replace(":", "_")
                found_file = None
                for f in os.listdir(sensor_folder):
                    if f.lower().endswith(".pcd") and sensor_identifier.lower() in f.lower() and pattern in f:
                        found_file = os.path.join(sensor_folder, f)
                        break
                if found_file:
                    if event_ts in local_events:
                        self.log(f"Removing file for sensor '{sensor_identifier}' at {event_ts}: {found_file}", tag="red")
                    else:
                        self.log(f"Removing file for sensor '{sensor_identifier}' at {event_ts}: {found_file}", tag="yellow")
                    if not dry_run:
                        try:
                            os.remove(found_file)
                        except Exception as e:
                            self.log(f"Failed to remove {found_file}: {e}")
                    total_removed += 1
                else:
                    self.log(f"No matching .pcd file found for sensor '{sensor_identifier}' at {event_ts}")
        self.log(f"Batch processing complete. Total affected .pcd files removed (or listed): {total_removed}")
    
    def simulate_vibration(self):
        # Simulation method remains unchanged.
        csv_file = filedialog.askopenfilename(
            title="Select an IMU CSV File for Simulation",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not csv_file:
            messagebox.showerror("Error", "No CSV file selected for simulation.")
            return
        try:
            df = pd.read_csv(csv_file)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read CSV file: {e}")
            return
        for col in ["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]:
            if col not in df.columns:
                messagebox.showerror("Error", f"CSV missing required column: {col}")
                return
        acc_x = df["Linear Acceleration X"].to_numpy()
        acc_y = df["Linear Acceleration Y"].to_numpy()
        acc_z = df["Linear Acceleration Z"].to_numpy()
        acc_mag = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        
        fs = 200
        n = len(acc_mag)
        if n < 2:
            messagebox.showerror("Error", "Not enough data to compute FFT.")
            return
        fft_vals = np.fft.rfft(acc_mag)
        fft_freq = np.fft.rfftfreq(n, d=1/fs)
        fft_amp = np.abs(fft_vals)
        if len(fft_amp) > 1:
            idx_peak = np.argmax(fft_amp[1:]) + 1
            f_dom = fft_freq[idx_peak]
        else:
            f_dom = 0
        if f_dom <= 0:
            messagebox.showerror("Error", "Unable to determine dominant vibration frequency.")
            return
        try:
            threshold = float(self.threshold_entry.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Invalid threshold value.")
            return
        a_amp = threshold * 9.81  # m/s²
        A = a_amp / ((2 * math.pi * f_dom)**2)  # displacement in m
        A_mm = A * 1000  # in mm
        
        sim_win = tk.Toplevel(self.root)
        sim_win.title("Vibration Simulation")
        
        fig = Figure(figsize=(12, 6), dpi=100)
        ax = fig.add_subplot(111)
        ax.set_xlim(-290, 290)
        ax.set_ylim(-122.5, 122.5)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_aspect('equal')
        ax.set_title(f"Simulated Vibration\nFreq: {f_dom:.2f} Hz, Amplitude: {A_mm:.3f} mm")
        
        rect_width = 77
        rect_height = 84.1
        rect_x = -rect_width / 2
        rect_y = -rect_height / 2
        rect = Rectangle((rect_x, rect_y), rect_width, rect_height, fc='blue')
        ax.add_patch(rect)
        
        canvas = FigureCanvasTkAgg(fig, master=sim_win)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        
        def update(frame):
            t = frame / 50.0
            y_disp = A_mm * math.sin(2 * math.pi * f_dom * t)
            rect.set_xy((rect_x, rect_y + y_disp))
            canvas.draw_idle()
            return rect,
        
        self.anim = FuncAnimation(fig, update, frames=np.arange(0, 250), interval=20, blit=False)
        self.log(f"Dominant frequency: {f_dom:.2f} Hz, displacement amplitude: {A_mm:.3f} mm")
        
if __name__ == "__main__":
    root = tk.Tk()
    app = IMUPCDFilterGUI(root)
    root.mainloop()
