#!/usr/bin/env python3
"""
Script: imu_vibration_filter.py

Description:
  Reads IMU data from a CSV file (assumed to have columns: timestamp, ax, ay, az),
  detects vibration events based on a deviation from the gravitational acceleration (9.81 m/s²),
  groups events into intervals, and then scans a directory of point cloud files to remove those
  whose timestamp (extracted from the filename) falls within any vibration interval (plus an extra time window).

Usage Example:
  python3 imu_vibration_filter.py --imu_file imu_data.csv --pc_dir /home/taixing/pointclouds/ --threshold 1.0 --time_window 1.0 --dry_run

Options:
  --imu_file: Path to the CSV file with IMU data.
  --pc_dir: Directory containing point cloud files.
  --threshold: Vibration threshold (m/s² deviation from 9.81). Default is 1.0.
  --time_window: Extra seconds before and after a detected event to consider a point cloud affected. Default is 1.0.
  --dry_run: If set, the script will list affected files without deleting them.
"""

import os
import math
import csv
import re
import argparse
from datetime import datetime

def parse_arguments():
    parser = argparse.ArgumentParser(description="Remove affected point clouds based on detected IMU vibration events.")
    parser.add_argument("--imu_file", required=True, help="Path to IMU CSV file.")
    parser.add_argument("--pc_dir", required=True, help="Directory containing point cloud files.")
    parser.add_argument("--threshold", type=float, default=1.0,
                        help="Vibration threshold (m/s² deviation from gravity 9.81).")
    parser.add_argument("--time_window", type=float, default=1.0,
                        help="Extra time (in seconds) before and after each event to mark as affected.")
    parser.add_argument("--dry_run", action="store_true",
                        help="If set, only list affected files without deleting them.")
    return parser.parse_args()

def detect_vibration_events(imu_file, threshold):
    """
    Detect vibration events from IMU data.
    Assumes a CSV file with header columns: timestamp, ax, ay, az.
    Returns a list of timestamps (float) when vibration is detected.
    """
    vibration_times = []
    with open(imu_file, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                timestamp = float(row["timestamp"])
                ax = float(row["ax"])
                ay = float(row["ay"])
                az = float(row["az"])
                magnitude = math.sqrt(ax**2 + ay**2 + az**2)
                # Compare against static gravity (9.81 m/s^2)
                deviation = abs(magnitude - 9.81)
                if deviation > threshold:
                    vibration_times.append(timestamp)
            except Exception as e:
                print(f"Error processing row {row}: {e}")
    return vibration_times

def merge_events(events, window):
    """
    Merge events that are within 'window' seconds of each other into intervals.
    Returns a list of (start, end) timestamp tuples.
    """
    if not events:
        return []
    events = sorted(events)
    intervals = []
    start = events[0]
    end = events[0]
    for t in events[1:]:
        if t - end <= window:
            end = t
        else:
            intervals.append((start, end))
            start = t
            end = t
    intervals.append((start, end))
    return intervals

def parse_pointcloud_timestamp(filename):
    """
    Extract timestamp from the point cloud filename.
    Assumes a filename format like: pointcloud_YYYYMMDD_HHMMSS.pcd
    Returns the timestamp as a float (epoch seconds) or None if parsing fails.
    """
    pattern = r"pointcloud_(\d{8})_(\d{6})"
    match = re.search(pattern, filename)
    if match:
        date_str = match.group(1)  # Expected format: YYYYMMDD
        time_str = match.group(2)  # Expected format: HHMMSS
        dt_str = f"{date_str} {time_str}"
        try:
            dt = datetime.strptime(dt_str, "%Y%m%d %H%M%S")
            return dt.timestamp()
        except Exception as e:
            print(f"Error parsing datetime from {dt_str}: {e}")
    return None

def remove_affected_pointclouds(pc_dir, vibration_intervals, time_window, dry_run=False):
    """
    Scans point cloud files in 'pc_dir' and deletes those whose extracted timestamp falls within
    any vibration interval (expanded by 'time_window' seconds on each side).
    """
    files_removed = []
    for filename in os.listdir(pc_dir):
        # Adjust file extensions as needed (e.g., .pcd, .ply)
        if filename.endswith(".pcd") or filename.endswith(".ply"):
            filepath = os.path.join(pc_dir, filename)
            ts = parse_pointcloud_timestamp(filename)
            if ts is None:
                print(f"Could not parse timestamp from {filename}, skipping.")
                continue
            affected = False
            for start, end in vibration_intervals:
                if (start - time_window) <= ts <= (end + time_window):
                    affected = True
                    break
            if affected:
                print(f"Affected file detected: {filename}")
                files_removed.append(filename)
                if not dry_run:
                    try:
                        os.remove(filepath)
                        print(f"Deleted file: {filename}")
                    except Exception as e:
                        print(f"Failed to delete {filename}: {e}")
    return files_removed

def main():
    args = parse_arguments()
    print("Detecting vibration events from IMU data...")
    events = detect_vibration_events(args.imu_file, args.threshold)
    print(f"Found {len(events)} vibration events.")
    
    intervals = merge_events(events, args.time_window)
    print(f"Merged into {len(intervals)} vibration intervals:")
    for start, end in intervals:
        start_dt = datetime.fromtimestamp(start).strftime("%Y-%m-%d %H:%M:%S")
        end_dt = datetime.fromtimestamp(end).strftime("%Y-%m-%d %H:%M:%S")
        print(f"  Interval: {start_dt} to {end_dt}")
    
    print("Scanning point cloud directory for affected files...")
    affected_files = remove_affected_pointclouds(args.pc_dir, intervals, args.time_window, dry_run=args.dry_run)
    print(f"Total affected files {'detected' if args.dry_run else 'removed'}: {len(affected_files)}")

if __name__ == "__main__":
    main()
