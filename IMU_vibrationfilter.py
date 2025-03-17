#!/usr/bin/env python3
"""
IMU Vibration PCD File Remover (GUI Version)

Description:
  This GUI application reads a CSV file containing IMU data recorded at 200 Hz.
  The CSV must contain these columns:
    - "Timestamp of last PCD": the filename (with extension) of the corresponding .pcd file.
    - "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z": acceleration values in g.
    
  Since each .pcd file is saved once per second (with about 200 IMU rows), the script groups the rows
  by the .pcd file name. For each group it calculates the magnitude of acceleration (in g) as:
  
      magnitude = sqrt((Linear Acceleration X)^2 + (Linear Acceleration Y)^2 + (Linear Acceleration Z)^2)
      
  It then computes the deviation from 1.0 g (i.e. |magnitude – 1.0|). If any row in that group shows a deviation
  greater than a user-set threshold, the corresponding .pcd file is considered affected and is removed
  (or just listed in dry-run mode).

Usage:
  - Select the IMU CSV file using the "Browse" button.
  - Select the folder containing the .pcd files.
  - Set the vibration threshold (in g). For example, 0.001 means if any reading deviates by more than 0.001 g from 1.0, the file is removed.
  - Check "Dry Run" to list affected files without deleting them.
  - Click "Process" to run the analysis.
"""

import os
import math
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
from datetime import datetime
import pandas as pd

class IMUPCDFilterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Vibration PCD File Remover")
        self.csv_file = ""
        self.pcd_folder = ""
        self.create_widgets()

    def create_widgets(self):
        # Frame for CSV file selection
        file_frame = tk.LabelFrame(self.root, text="IMU CSV File")
        file_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(file_frame, text="CSV File:").grid(row=0, column=0, sticky="w")
        self.csv_entry = tk.Entry(file_frame, width=50)
        self.csv_entry.grid(row=0, column=1, padx=5, pady=2)
        tk.Button(file_frame, text="Browse...", command=self.browse_csv).grid(row=0, column=2, padx=5)

        # Frame for PCD folder selection
        folder_frame = tk.LabelFrame(self.root, text="PCD Folder")
        folder_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(folder_frame, text="Folder:").grid(row=0, column=0, sticky="w")
        self.folder_entry = tk.Entry(folder_frame, width=50)
        self.folder_entry.grid(row=0, column=1, padx=5, pady=2)
        tk.Button(folder_frame, text="Browse...", command=self.browse_folder).grid(row=0, column=2, padx=5)

        # Frame for parameters
        param_frame = tk.LabelFrame(self.root, text="Parameters")
        param_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(param_frame, text="Vibration Threshold (g):").grid(row=0, column=0, sticky="w")
        self.threshold_entry = tk.Entry(param_frame, width=10)
        self.threshold_entry.insert(0, "0.001")
        self.threshold_entry.grid(row=0, column=1, padx=5, pady=2, sticky="w")
        
        self.dry_run_var = tk.BooleanVar(value=True)
        tk.Checkbutton(param_frame, text="Dry Run (do not delete files)", variable=self.dry_run_var).grid(row=0, column=2, padx=5, sticky="w")

        # Process Button
        tk.Button(self.root, text="Process", command=self.process_data).pack(pady=5)

        # Output log display - now resizable
        self.log_text = scrolledtext.ScrolledText(self.root, width=80, height=15)
        self.log_text.pack(padx=10, pady=10, fill="both", expand=True)

    def browse_csv(self):
        file_path = filedialog.askopenfilename(title="Select IMU CSV File", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if file_path:
            self.csv_file = file_path
            self.csv_entry.delete(0, tk.END)
            self.csv_entry.insert(0, file_path)

    def browse_folder(self):
        folder_path = filedialog.askdirectory(title="Select PCD Folder")
        if folder_path:
            self.pcd_folder = folder_path
            self.folder_entry.delete(0, tk.END)
            self.folder_entry.insert(0, folder_path)

    def log(self, message):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)

    def process_data(self):
        self.log_text.delete(1.0, tk.END)  # clear previous log
        if not self.csv_file:
            messagebox.showerror("Error", "Please select an IMU CSV file.")
            return
        if not self.pcd_folder:
            messagebox.showerror("Error", "Please select a PCD folder.")
            return

        try:
            threshold = float(self.threshold_entry.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Invalid threshold value.")
            return

        dry_run = self.dry_run_var.get()
        self.log("Reading CSV file...")
        try:
            df = pd.read_csv(self.csv_file)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read CSV file: {e}")
            return

        # Check for required columns
        required = ["Timestamp of last PCD", "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]
        for col in required:
            if col not in df.columns:
                messagebox.showerror("Error", f"CSV missing required column: {col}")
                return

        self.log("Grouping IMU data by 'Timestamp of last PCD' ...")
        groups = df.groupby("Timestamp of last PCD")
        files_to_remove = []
        total_groups = groups.ngroups
        self.log(f"Found {total_groups} groups (PCD files) in CSV.")

        for file_name, group in groups:
            # Compute the acceleration magnitude and deviation from 1.0 g for each row.
            deviations = group[["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]].apply(
                lambda row: abs(math.sqrt(row["Linear Acceleration X"]**2 +
                                           row["Linear Acceleration Y"]**2 +
                                           row["Linear Acceleration Z"]**2) - 1.0), axis=1)
            if (deviations > threshold).any():
                files_to_remove.append(file_name)

        self.log(f"Found {len(files_to_remove)} affected .pcd files based on threshold {threshold} g.")

        for file_name in files_to_remove:
            full_path = os.path.join(self.pcd_folder, file_name)
            if os.path.exists(full_path):
                self.log(f"Removing file: {full_path}")
                if not dry_run:
                    try:
                        os.remove(full_path)
                    except Exception as e:
                        self.log(f"Failed to remove {full_path}: {e}")
            else:
                self.log(f"File not found: {full_path}")

        self.log("Processing complete.")

if __name__ == "__main__":
    root = tk.Tk()
    app = IMUPCDFilterGUI(root)
    root.mainloop()
