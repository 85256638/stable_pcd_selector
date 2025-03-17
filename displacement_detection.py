import sys
import os
import struct
import numpy as np
import open3d as o3d
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QFileDialog,
    QTextEdit, QPlainTextEdit, QLabel
)

# 兼容性处理：解决 pypcd 内部引用 cStringIO 的问题（如果有 pypcd 使用场景）
try:
    import cStringIO
except ImportError:
    import io
    import sys
    sys.modules["cStringIO"] = io

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

class PCDComparator(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PCD 位移对比工具")
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        btn_layout = QHBoxLayout()
        self.select_btn = QPushButton("选择多个 PCD 文件")
        self.select_btn.clicked.connect(self.select_files)
        btn_layout.addWidget(self.select_btn)
        layout.addLayout(btn_layout)

        label = QLabel("文件路径（每行一个，可复制粘贴）：")
        layout.addWidget(label)

        self.path_text = QPlainTextEdit()
        layout.addWidget(self.path_text)

        self.compare_btn = QPushButton("对比位移")
        self.compare_btn.clicked.connect(self.compare_displacement)
        layout.addWidget(self.compare_btn)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        layout.addWidget(self.result_text)

    def select_files(self):
        files, _ = QFileDialog.getOpenFileNames(self, "选择 PCD 文件", "", "PCD Files (*.pcd)")
        if files:
            current_text = self.path_text.toPlainText()
            if current_text:
                current_text += "\n"
            current_text += "\n".join(files)
            self.path_text.setPlainText(current_text)

    def format_filename(self, file_path):
        """
        返回整个文件路径，其中仅文件名部分用深绿色高亮显示。
        """
        directory = os.path.dirname(file_path)
        filename = os.path.basename(file_path)
        # 如果有目录，则拼接目录 + "/" + 高亮后的文件名
        if directory:
            return f"{directory}/{'<span style=\'color: darkgreen;\'>'}{filename}{'</span>'}"
        else:
            return f"<span style='color: darkgreen;'>{filename}</span>"

    def compare_displacement(self):
        centroids = []
        result_lines = []
        text = self.path_text.toPlainText()
        file_list = [line.strip() for line in text.splitlines() if line.strip()]
        if not file_list:
            result_lines.append("未输入任何 PCD 文件路径。")
            self.result_text.setHtml("<pre>" + "\n".join(result_lines) + "</pre>")
            return

        for file_path in file_list:
            try:
                pcd = o3d.io.read_point_cloud(file_path)
                pts = np.asarray(pcd.points)
                if pts.size == 0:
                    raise ValueError("点云数据为空")
            except Exception as e:
                try:
                    with open(file_path, 'rb') as f:
                        content = f.read()
                    pts = parse_pcd(content)
                    if pts.size == 0:
                        raise ValueError("点云数据为空")
                except Exception as e2:
                    result_lines.append(f"读取文件 {self.format_filename(file_path)} 时出错: {str(e2)}")
                    continue

            centroid = pts.mean(axis=0)
            centroids.append((file_path, centroid))
            result_lines.append(f"文件 {self.format_filename(file_path)} 质心: {centroid}")

        if len(centroids) < 2:
            result_lines.append("有效点云数量不足，无法进行对比。")
            self.result_text.setHtml("<pre>" + "\n".join(result_lines) + "</pre>")
            return

        displacement_list = []
        n = len(centroids)
        for i in range(n):
            for j in range(i + 1, n):
                d = np.linalg.norm(centroids[i][1] - centroids[j][1])
                displacement_list.append((d, centroids[i][0], centroids[j][0]))
        
        displacement_list.sort(key=lambda x: x[0])
        
        result_lines.append("<br>各文件之间的位移（欧氏距离，按从小到大排序）：")
        for d, file1, file2 in displacement_list:
            result_lines.append(f"{self.format_filename(file1)} 与 {self.format_filename(file2)}: {d:.6f}")
        
        threshold = 1e-3
        consistent = all(d < threshold for d, _, _ in displacement_list)
        if consistent:
            result_lines.append("<br>各点云间无显著位移（位移均低于阈值）。")
        else:
            result_lines.append("<br>部分点云间存在显著位移。")
        
        html_result = "<pre>" + "\n".join(result_lines) + "</pre>"
        self.result_text.setHtml(html_result)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PCDComparator()
    window.show()
    sys.exit(app.exec_())
