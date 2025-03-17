#!/usr/bin/env python3
"""
Stable PCD Selector

本脚本整合了：
1. 基于 IMU 数据的震动快速粗筛，剔除震动时刻对应的 PCD 文件；
2. 基于质心位移标准差的滑动窗口筛选，选出一段相对稳定的连续帧；
3. 在该稳定窗口内，采用 ICP 精细筛选，选出最终稳定的若干帧。

最终将选中的 PCD 文件复制到文件夹 “refined_pcd” 中。

依赖库：open3d, numpy, pandas
"""

import os
import glob
import re
import math
import shutil
import struct
import numpy as np
import pandas as pd
import open3d as o3d

# ============== 参数配置（根据实际情况调整） ==============
# 注意：使用原始字符串防止转义问题
imu_folder = r"E:\ChengduNAS\P003_激光雷达测厚\technical\250311_鞍山4台雷达_3月11号之前收集到的点云\Cam-2-Trigger-Lidar_DATASAVES\pointclouds\session_2025-03-11_02-13-20_to_2025-03-11_02-14-36_倾斜45_20号\lidar_farleft_2"
pcd_folder = r"E:\ChengduNAS\P003_激光雷达测厚\technical\250311_鞍山4台雷达_3月11号之前收集到的点云\Cam-2-Trigger-Lidar_DATASAVES\pointclouds\session_2025-03-11_02-13-20_to_2025-03-11_02-14-36_倾斜45_20号\lidar_farleft_2"
output_dir = r"E:\ChengduNAS\P003_激光雷达测厚\technical\250311_鞍山4台雷达_3月11号之前收集到的点云\Cam-2-Trigger-Lidar_DATASAVES\pointclouds\session_2025-03-11_02-13-20_to_2025-03-11_02-14-36_倾斜45_20号\lidar_farleft_2\refined_pcd"

imu_threshold = 0.12        # IMU 震动阈值：每帧所有加速度（X,Y,Z）幅值偏离 1g 的最大允许值
centroid_window_size = 12      # 滑动窗口大小（帧数），用于质心稳定性分析
top_n_stable = 6              # 最终通过 ICP 筛选后选取的稳定帧数量
icp_distance = 0.04           # ICP 配准的最大对应点距离
# ============================================================

os.makedirs(output_dir, exist_ok=True)

# ANSI 控制码（仅用于统计信息数值）
red = "\033[31m"
reset = "\033[0m"
separator = "=" * 69

# -----------------------------------------------------------------------------
# 辅助函数：解析二进制 PCD 文件（仅支持包含 x,y,z 字段）
# 参考 displacement_detection.py
# -----------------------------------------------------------------------------
def parse_pcd(content):
    """
    简易解析二进制格式 PCD 文件，仅支持包含 x, y, z 字段的情况。
    """
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
    for field, count in zip(fields, counts):
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

# =============================================================================
# Step 1: IMU 震动快速粗筛
# =============================================================================
print("Step 1: IMU Vibration Filtering")
imu_csv_candidates = glob.glob(os.path.join(imu_folder, "imu_data_*.csv"))
if not imu_csv_candidates:
    print("未在指定目录中找到以 'imu_data_' 开头的 CSV 文件")
    exit(1)
imu_csv_path = imu_csv_candidates[0]
print(f"使用的 IMU CSV 文件: {imu_csv_path}")

try:
    imu_df = pd.read_csv(imu_csv_path)
except Exception as e:
    print(f"Error reading IMU CSV: {e}")
    exit(1)

required_cols = ["Timestamp of last PCD", "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]
for col in required_cols:
    if col not in imu_df.columns:
        print(f"IMU CSV 缺少必需的列: {col}")
        exit(1)

stable_timestamps = []
grouped = imu_df.groupby("Timestamp of last PCD")
for ts, group in grouped:
    deviations = group[["Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"]].apply(
        lambda row: abs(math.sqrt(row["Linear Acceleration X"]**2 +
                                   row["Linear Acceleration Y"]**2 +
                                   row["Linear Acceleration Z"]**2) - 1.0),
        axis=1
    )
    if (deviations < imu_threshold).all():
        stable_timestamps.append(ts.strip())
print(f"基于 IMU 筛选得到的稳定时间戳: {stable_timestamps}")
print(separator)

# =============================================================================
# Step 2: 从 pcd_folder 中选取候选 PCD 文件（IMU 筛选结果）
# =============================================================================
print("Step 2: Selecting candidate PCD files from folder")
pcd_files = sorted(glob.glob(os.path.join(pcd_folder, "*.pcd")))
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
    print("经过 IMU 筛选后没有找到候选 PCD 文件。")
    exit(1)
total_pcd = len(pcd_files)
candidate_count = len(candidate_pcd_files)
excluded_imufilter = total_pcd - candidate_count
print(f"{red}经过 IMU 筛选: 总共 {total_pcd} 个 PCD, 排除了 {excluded_imufilter} 个, 保留了 {candidate_count} 个 PCD{reset}")
print("经过 IMU 筛选后的候选 PCD 文件：")
for f in candidate_pcd_files:
    print("  ", f)
print(separator)

# =============================================================================
# Step 3: 质心位移稳定性筛选（基于 displacement_detection.py 的思路）
# =============================================================================
print("Step 3: Centroid Displacement Screening using sliding window")
centroids = []
for f in candidate_pcd_files:
    pcd = robust_read_point_cloud(f)
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        print(f"警告：PCD 文件 {f} 没有点云数据。")
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
print(f"{red}经过质心位移稳定性筛选: 总共 {candidate_count} 个候选 PCD, 排除了 {excluded_centroid} 个, 保留了 {len(stable_window_files)} 个 PCD{reset}")

# 详细输出稳定窗口内信息（关键数值部分用红色高亮）
window_centroids = centroids[best_window_idx: best_window_idx + window_size]
window_mean = np.mean(window_centroids, axis=0)
print(f"{red}[详细信息] 稳定窗口内各帧质心及其偏离信息：{reset}")
print(f"{red}窗口均值: {window_mean}{reset}")
for idx, c in enumerate(window_centroids):
    deviation = np.linalg.norm(c - window_mean)
    print(f"帧 {best_window_idx + idx} - 文件: {candidate_pcd_files[best_window_idx + idx]}")
    print(f"{red}  质心: {c}, 偏离窗口均值: {deviation:.6f}{reset}")
print(f"{red}窗口内偏离标准差: {min_stability:.6f}{reset}")
print(separator)

# =============================================================================
# Step 4: ICP 精细筛选
# =============================================================================
print("Step 4: ICP Fine Screening within the stable window")
# 选择稳定窗口内与窗口均值偏离最小的帧作为参考点云
window_centroids = centroids[best_window_idx: best_window_idx + window_size]
window_mean = np.mean(window_centroids, axis=0)
deviations = [np.linalg.norm(c - window_mean) for c in window_centroids]
ref_idx_relative = np.argmin(deviations)
ref_file = stable_window_files[ref_idx_relative]
ref_pcd = robust_read_point_cloud(ref_file)
print(f"参考 PCD 文件 (偏离窗口均值最小的): {ref_file}")
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
print(f"{red}经过 ICP 精细筛选: 总共 {len(stable_window_files)} 个 PCD, 排除了 {excluded_icp} 个, 保留了 {len(final_selected_files)} 个 PCD{reset}")
print("基于 ICP 筛选最终选中的 PCD 文件：")
for f, fitness in icp_results[:top_n_stable]:
    print(f"  {f}: fitness={fitness:.6f}")
print(separator)

# =============================================================================
# Step 5: 复制选中的 PCD 文件到输出目录 refined_pcd
# =============================================================================
print(f"正在将选中的 PCD 文件复制到目录: {output_dir}")
for f in final_selected_files:
    shutil.copy(f, os.path.join(output_dir, os.path.basename(f)))
print("完成。稳定的 PCD 文件已复制到:", output_dir)
