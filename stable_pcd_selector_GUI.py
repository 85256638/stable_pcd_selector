#!/usr/bin/env python3
"""
Stable PCD Selector with GUI and Shared Vibration Timestamps

本项目通过 GUI 提供以下功能：
1. 用户选择一个基目录（session 文件夹），程序自动递归搜索该目录下所有子文件夹（各个 LiDAR）。
2. 每个子文件夹中应包含以 "imu_data_" 开头的 CSV 文件和 PCD 文件。
3. 用户可选择是否使用“共享震动时间戳”：
   - 若开启，则将所有子文件夹的 IMU 数据进行合并（取交集），作为全局共享的稳定时间戳，并在日志中显示加载的 IMU CSV 文件名及排除的震动时间戳（红色高亮）。
   - 若关闭，则每个子文件夹独立处理。
4. 程序依次执行：
   - �� 基于 IMU 数据进行震动筛选；
   - �� 利用时间戳匹配筛选候选 PCD 文件；
   - �� 质心稳定性分析（滑动窗口计算，并详细输出窗口均值、各帧质心及其偏离）；
   - �� ICP 精细筛选（选择窗口内与窗口均值偏离最小的帧作为参考，对窗口内其它帧进行 ICP 配准，并按 fitness 排序）；
   - �� 将最终筛选出的 PCD 文件复制到各子文件夹下的 “refined_pcd” 目录中。
5. 所有日志信息输出到 GUI 的日志框中，日志框支持颜色（通过 tag 实现），并可调整大小。

依赖库：tkinter, open3d, numpy, pandas
"""

import os
import glob
import re
import math
import shutil
import struct
import threading
import numpy as np
import pandas as pd
import open3d as o3d
import tkinter as tk
from tkinter import filedialog, scrolledtext

# ============== 参数配置（可根据实际情况调整） ==============
imu_threshold = 0.2         # IMU 震动阈值，– 较低的阈值意味着对数据稳定性要求更高，最终挑选出的点云数据会更稳定，但可能不够充分；– 较高的阈值则允许更多数据进入后续处理流程，但是可能会降低数据整体的稳定性和匹配精度。
centroid_window_size = 18    # 滑动窗口大小（帧数）
top_n_stable = 9             # ICP 筛选后最终保留的帧数
icp_distance = 0.04          # ICP 配准的最大对应点距离
separator = "=" * 69
# ============================================================

# -----------------------------------------------------------------------------
# 辅助函数：解析二进制 PCD 文件（仅支持包含 x,y,z 字段）
# -----------------------------------------------------------------------------
def parse_pcd(content):
    lines = content.split(b'\n')
    header_lines = []
    data_line_index = None
    for i, line in enumerate(lines):
        if line.strip() == b'':
            continue
        header_lines.append(line)
        if line.upper().startswith(b'DATA'):
            data_line_index = i
            break
    if data_line_index is None:
        raise ValueError("未找到 DATA 行")
    
    header_str = b'\n'.join(header_lines).decode('utf-8', errors='ignore')
    header = {}
    for line in header_str.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            key = parts[0].upper()
            header[key] = parts[1:]
    for key in ['FIELDS', 'SIZE', 'TYPE', 'COUNT', 'POINTS', 'DATA']:
        if key not in header:
            raise ValueError(f"缺少 {key} 字段")
    if header['DATA'][0].lower() != 'binary':
        raise ValueError("仅支持 DATA 为 binary 的 PCD 文件")
    
    fields = header['FIELDS']
    sizes = list(map(int, header['SIZE']))
    types = header['TYPE']
    counts = list(map(int, header['COUNT']))
    points = int(header['POINTS'][0])
    
    record_size = sum(s * c for s, c in zip(sizes, counts))
    
    fmt = ''
    for size, typ, count in zip(sizes, types, counts):
        if typ.upper() == 'F':
            if size == 4:
                fmt += 'f' * count
            elif size == 8:
                fmt += 'd' * count
            else:
                raise ValueError("不支持的浮点数大小")
        elif typ.upper() == 'I':
            fmt += 'i' * count
        elif typ.upper() == 'U':
            fmt += 'I' * count
        else:
            raise ValueError(f"未知数据类型 {typ}")
    fmt = '<' + fmt
    data_start = content.find(b'\n', content.find(b'DATA')) + 1
    binary_data = content[data_start:]
    if len(binary_data) < points * record_size:
        raise ValueError("二进制数据长度不足")
    
    records = []
    for i in range(points):
        start = i * record_size
        end = start + record_size
        record = struct.unpack(fmt, binary_data[start:end])
        records.append(record)
    records = np.array(records)
    
    indices = []
    pos = 0
    for field, count in zip(header['FIELDS'], counts):
        if field in ['x', 'y', 'z']:
            indices.extend(range(pos, pos + count))
        pos += count
    if len(indices) < 3:
        raise ValueError("未找到足够的 x, y, z 字段")
    xyz = records[:, indices]
    if xyz.shape[1] > 3:
        xyz = xyz[:, :3]
    return xyz

# -----------------------------------------------------------------------------
# 通用函数：稳健读取 PCD 文件
# -----------------------------------------------------------------------------
def robust_read_point_cloud(pcd_file):
    try:
        pcd = o3d.io.read_point_cloud(pcd_file)
    except UnicodeDecodeError:
        with open(pcd_file, "rb") as fin:
            content = fin.read()
        pts = parse_pcd(content)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd

# -----------------------------------------------------------------------------
# GUI 应用部分
# -----------------------------------------------------------------------------
class StablePCDSelectorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Stable PCD Selector")
        self.geometry("900x600")
        self.create_widgets()

    def create_widgets(self):
        # 顶部区域：文件夹选择和选项
        top_frame = tk.Frame(self)
        top_frame.pack(padx=10, pady=5, fill="x")
        tk.Label(top_frame, text="Base Folder:").pack(side="left")
        self.folder_var = tk.StringVar()
        self.folder_entry = tk.Entry(top_frame, textvariable=self.folder_var)
        self.folder_entry.pack(side="left", fill="x", expand=True, padx=5)
        tk.Button(top_frame, text="Browse...", command=self.browse_folder).pack(side="left")
        tk.Button(top_frame, text="Run", command=self.run_workflow).pack(side="left", padx=5)
        tk.Button(top_frame, text="Clear Log", command=self.clear_log).pack(side="left", padx=5)
        # 共享震动时间戳复选框
        self.share_vibration_var = tk.BooleanVar()
        tk.Checkbutton(top_frame, text="共享震动时间戳", variable=self.share_vibration_var).pack(side="left", padx=5)
        
        # Log 显示区域（可调整大小）
        self.log_text = scrolledtext.ScrolledText(self, wrap="word")
        self.log_text.pack(padx=10, pady=5, fill="both", expand=True)
        # 配置颜色标签
        self.log_text.tag_config("red", foreground="red")
        self.log_text.tag_config("blue", foreground="blue")

    def browse_folder(self):
        folder = filedialog.askdirectory(title="Select Base Folder")
        if folder:
            self.folder_var.set(folder)

    def clear_log(self):
        self.log_text.delete("1.0", tk.END)

    def log(self, message, tag=None):
        if tag:
            self.log_text.insert(tk.END, message + "\n", tag)
        else:
            self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)

    def run_workflow(self):
        base_folder = self.folder_var.get().strip()
        if not base_folder or not os.path.isdir(base_folder):
            self.log("请选择有效的基目录！", tag="red")
            return
        self.log("开始处理...")
        threading.Thread(target=self.workflow_thread, args=(base_folder,), daemon=True).start()

    def workflow_thread(self, base_folder):
        if self.share_vibration_var.get():
            self.log(separator)
            self.log("共享震动时间戳模式开启")
            subfolders = [os.path.join(base_folder, d) for d in os.listdir(base_folder) if os.path.isdir(os.path.join(base_folder, d))]
            valid_subfolders = []
            imu_csv_list = []
            all_stable_ts = []
            for folder in subfolders:
                files = os.listdir(folder)
                if any(f.lower().startswith("imu_data_") and f.lower().endswith(".csv") for f in files) and any(f.lower().endswith(".pcd") for f in files):
                    valid_subfolders.append(folder)
                    imu_csv_candidates = [os.path.join(folder, f) for f in files if f.lower().startswith("imu_data_") and f.lower().endswith(".csv")]
                    if imu_csv_candidates:
                        imu_csv_list.append(imu_csv_candidates[0])
                        try:
                            imu_df = pd.read_csv(imu_csv_candidates[0])
                        except Exception as e:
                            self.log(f"Error reading IMU CSV {imu_csv_candidates[0]}: {e}", tag="red")
                            continue
                        stable_ts = set()
                        for ts, group in imu_df.groupby("Timestamp of last PCD"):
                            deviations = group[["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]].apply(
                                lambda row: abs(math.sqrt(row["Linear Acceleration X"]**2 +
                                                           row["Linear Acceleration Y"]**2 +
                                                           row["Linear Acceleration Z"]**2) - 1.0),
                                axis=1
                            )
                            if (deviations < imu_threshold).all():
                                stable_ts.add(ts.strip())
                        all_stable_ts.append(stable_ts)
            # 输出加载的 IMU CSV 文件名数量和列表
            self.log(f"加载了 {len(imu_csv_list)} 个 IMU CSV 文件", tag="blue")
            for imu_file in imu_csv_list:
                self.log(f"  {imu_file}", tag="blue")
            if not all_stable_ts:
                self.log("未能获得任何稳定时间戳。", tag="red")
                return
            shared_stable_ts = all_stable_ts[0]
            for ts_set in all_stable_ts[1:]:
                shared_stable_ts = shared_stable_ts.intersection(ts_set)
            # 计算排除的时间戳
            union_ts = set()
            for ts_set in all_stable_ts:
                union_ts = union_ts.union(ts_set)
            excluded_ts = union_ts - shared_stable_ts
            self.log(f"共享稳定时间戳: {sorted(shared_stable_ts)}", tag="blue")
            self.log(f"排除震动时间戳: {sorted(excluded_ts)}", tag="red")
            for folder in valid_subfolders:
                self.process_folder(folder, shared_stable_ts)
        else:
            for root, dirs, files in os.walk(base_folder):
                if any(f.lower().startswith("imu_data_") and f.lower().endswith(".csv") for f in files) and any(f.lower().endswith(".pcd") for f in files):
                    self.log(separator)
                    self.log(f"处理目录: {root}")
                    imu_csv_candidates = [f for f in files if f.lower().startswith("imu_data_") and f.lower().endswith(".csv")]
                    if not imu_csv_candidates:
                        continue
                    imu_csv_path = os.path.join(root, imu_csv_candidates[0])
                    self.log(f"使用的 IMU CSV 文件: {imu_csv_path}")
                    try:
                        imu_df = pd.read_csv(imu_csv_path)
                    except Exception as e:
                        self.log(f"Error reading IMU CSV: {e}", tag="red")
                        continue
                    required_cols = ["Timestamp of last PCD", "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]
                    for col in required_cols:
                        if col not in imu_df.columns:
                            self.log(f"IMU CSV 缺少必需的列: {col}", tag="red")
                            continue
                    stable_timestamps = []
                    for ts, group in imu_df.groupby("Timestamp of last PCD"):
                        deviations = group[["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]].apply(
                            lambda row: abs(math.sqrt(row["Linear Acceleration X"]**2 +
                                                       row["Linear Acceleration Y"]**2 +
                                                       row["Linear Acceleration Z"]**2) - 1.0),
                            axis=1
                        )
                        if (deviations < imu_threshold).all():
                            stable_timestamps.append(ts.strip())
                    self.log(f"基于 IMU 筛选得到的稳定时间戳: {stable_timestamps}")
                    self.process_folder(root, set(stable_timestamps))
            self.log(separator)
            self.log("所有符合条件的目录处理完毕。")
        self.log(separator)
        self.log("处理结束。")

    def process_folder(self, folder, stable_timestamps):
        pcd_files = sorted(glob.glob(os.path.join(folder, "*.pcd")))
        candidate_pcd_files = []
        timestamp_pattern = re.compile(r"(\d{4}-\d{2}-\d{2})_(\d{2})_(\d{2})_(\d{2})")
        for f in pcd_files:
            basename = os.path.splitext(os.path.basename(f))[0]
            m = timestamp_pattern.search(basename)
            if m:
                ts_formatted = f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}"
                if ts_formatted in stable_timestamps:
                    candidate_pcd_files.append(f)
        if not candidate_pcd_files:
            self.log("经过 IMU 筛选后没有找到候选 PCD 文件。", tag="red")
            return
        total_pcd = len(pcd_files)
        candidate_count = len(candidate_pcd_files)
        excluded_imufilter = total_pcd - candidate_count
        self.log(f"经过 IMU 筛选: 总共 {total_pcd} 个 PCD, 排除了 {excluded_imufilter} 个, 保留了 {candidate_count} 个 PCD", tag="red")
        self.log("候选 PCD 文件：")
        for f in candidate_pcd_files:
            self.log("  " + f)
        self.log(separator)
        # Step 3: 质心稳定性筛选
        centroids = []
        for f in candidate_pcd_files:
            pcd = robust_read_point_cloud(f)
            pts = np.asarray(pcd.points)
            if pts.size == 0:
                self.log(f"警告：PCD 文件 {f} 没有点云数据。", tag="red")
                centroids.append(np.array([np.nan, np.nan, np.nan]))
            else:
                centroids.append(pts.mean(axis=0))
        centroids = np.array(centroids)
        min_stability = np.inf
        best_window_idx = 0
        n = len(centroids)
        if n < centroid_window_size:
            best_window_idx = 0
            window_size = n
        else:
            window_size = centroid_window_size
            for i in range(n - window_size + 1):
                window_centroids = centroids[i:i+window_size]
                window_mean = np.mean(window_centroids, axis=0)
                distances = np.linalg.norm(window_centroids - window_mean, axis=1)
                stability = np.std(distances)
                if stability < min_stability:
                    min_stability = stability
                    best_window_idx = i
        stable_window_files = candidate_pcd_files[best_window_idx: best_window_idx + window_size]
        excluded_centroid = candidate_count - len(stable_window_files)
        self.log(f"经过质心位移稳定性筛选: 总共 {candidate_count} 个候选 PCD, 排除了 {excluded_centroid} 个, 保留了 {len(stable_window_files)} 个 PCD", tag="red")
        window_centroids = centroids[best_window_idx: best_window_idx + window_size]
        window_mean = np.mean(window_centroids, axis=0)
        self.log("[详细信息] 稳定窗口内各帧质心及偏离信息：", tag="red")
        self.log(f"窗口均值: {window_mean}", tag="red")
        for idx, c in enumerate(window_centroids):
            deviation = np.linalg.norm(c - window_mean)
            self.log(f"帧 {best_window_idx + idx} - 文件: {candidate_pcd_files[best_window_idx + idx]}")
            self.log(f"  质心: {c}, 偏离窗口均值: {deviation:.6f}", tag="red")
        self.log(f"窗口内偏离标准差: {min_stability:.6f}", tag="red")
        self.log(separator)
        # Step 4: ICP 精细筛选
        deviations = [np.linalg.norm(c - window_mean) for c in window_centroids]
        ref_idx_relative = int(np.argmin(deviations))
        ref_file = stable_window_files[ref_idx_relative]
        ref_pcd = robust_read_point_cloud(ref_file)
        self.log(f"参考 PCD 文件 (偏离窗口均值最小的): {ref_file}")
        icp_results = []
        for f in stable_window_files:
            source_pcd = robust_read_point_cloud(f)
            source_down = source_pcd.voxel_down_sample(voxel_size=0.005)
            ref_down = ref_pcd.voxel_down_sample(voxel_size=0.005)
            reg = o3d.pipelines.registration.registration_icp(
                source_down, ref_down, icp_distance, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            icp_results.append((f, reg.fitness))
        icp_results.sort(key=lambda x: x[1], reverse=True)
        final_selected_files = [f for f, fitness in icp_results[:top_n_stable]]
        excluded_icp = len(stable_window_files) - len(final_selected_files)
        self.log(f"经过 ICP 精细筛选: 总共 {len(stable_window_files)} 个 PCD, 排除了 {excluded_icp} 个, 保留了 {len(final_selected_files)} 个 PCD", tag="red")
        self.log("基于 ICP 筛选最终选中的 PCD 文件：")
        for f, fitness in icp_results[:top_n_stable]:
            self.log(f"  {f}: fitness={fitness:.6f}")
        self.log(separator)
        # Step 5: 复制结果
        output_dir_local = os.path.join(folder, "refined_pcd")
        os.makedirs(output_dir_local, exist_ok=True)
        for f in final_selected_files:
            shutil.copy(f, os.path.join(output_dir_local, os.path.basename(f)))
        self.log(f"完成。稳定的 PCD 文件已复制到: {output_dir_local}")

if __name__ == "__main__":
    app = StablePCDSelectorApp()
    app.mainloop()
