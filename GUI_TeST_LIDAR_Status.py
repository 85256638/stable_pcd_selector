import tkinter as tk
import socket
import threading
import time
import struct
import select
import crcmod

# ----------------------
# CRC HELPER FUNCTIONS
# ----------------------
crc16_func = crcmod.mkCrcFun(0x11021, rev=True, initCrc=0x4C49, xorOut=0x0000)
crc32_func = crcmod.mkCrcFun(0x104C11DB7, rev=True, initCrc=0x564F580A, xorOut=0xFFFFFFFF)

def compute_crc16(data: bytes) -> int:
    return crc16_func(data)

def compute_crc32(data: bytes) -> int:
    return crc32_func(data)

# ----------------------
# CONSTANTS
# ----------------------
BROADCAST_PORT = 55000
LIDAR_CMD_PORT = 65000

HOST_IP = "192.168.1.33"
CMD_PORT = 50001
DATA_PORT = 50002

DEV_TYPE_MAP = {
    0: "Livox Hub",
    1: "Mid-40",
    2: "Tele-15",
    3: "Horizon",
    6: "Mid-70",
    7: "Avia"
}

# Handshake and Heartbeat data
HANDSHAKE_HEX = "aa011900000000dc580001c0a8012152c351c352c363925b8b"
HEARTBEAT_SEQUENCE = [
    "aa010f0000000004d7000338ba8d0c",
    "aa010f00000100dcce00ff23decd83",
    "aa010f00000200b4e40002f63e1d0d",
    "aa010f000003006cfd01029bd54d2f",
    "aa010f0000040064b00009963b7701",
    "aa010f00000500bca90105d09c912a",
    "aa010f00000600d483010788934389",
    "aa010f000007000c9a0109a364b055",
    "aa011200000800f0d1000c010200a77f8a5c"
]

def decode_broadcast(data: bytes):
    """Look for a 21-byte payload starting with 0x00,0x00."""
    for i in range(len(data) - 20):
        if data[i] == 0x00 and data[i+1] == 0x00:
            payload = data[i:i+21]
            code_bytes = payload[2:18]
            broadcast_code = code_bytes.split(b'\x00')[0].decode('ascii', errors='ignore')
            dev_type = payload[18]
            return {
                'cmd_set': payload[0],
                'cmd_id': payload[1],
                'broadcast_code': broadcast_code,
                'dev_type': dev_type
            }
    return None

# For LiDAR set mode
MODE_MAP = {
    "Normal": 0,
    "Power-saving": 1,
    "Standby": 2
}

# A small helper for decoding ret_code, work_state, etc.
RET_CODE_MAP = {
    0x00: "Success",
    0x01: "Fail"
    # 0x02 might appear in other contexts, e.g. "Switching", if doc says so
}

WORK_STATE_MAP = {
    0x00: "Initializing",
    0x01: "Normal",
    0x02: "Power-Saving",
    0x03: "Standby",
    0x04: "Error"
}

class LidarRow:
    def __init__(self, parent, name, ip, row, log_callback):
        self.name = name
        self.ip = ip
        self.log_callback = log_callback

        self.last_broadcast_time = 0.0
        self.broadcast_code = None
        self.connected = False
        self.heartbeat_running = False

        # Sockets
        self.cmd_socket = None
        self.data_socket = None

        # Threads
        self.connect_thread = None
        self.heartbeat_thread = None
        self.data_thread = None
        self.data_running = False

        # Connect toggle
        self.conn_var = tk.IntVar(value=0)
        self.toggle_btn = tk.Checkbutton(parent, text="Connect", variable=self.conn_var,
                                         command=self.on_toggle)
        self.toggle_btn.grid(row=row, column=4, padx=5, pady=5)

        # Basic UI
        self.label_name = tk.Label(parent, text=name, width=12, anchor="w")
        self.label_name.grid(row=row, column=0, padx=5, pady=5)

        self.indicator = tk.Label(parent, text=" ", width=3, bg="gray")
        self.indicator.grid(row=row, column=1, padx=5, pady=5)

        self.label_ip = tk.Label(parent, text=ip, width=15, anchor="w")
        self.label_ip.grid(row=row, column=2, padx=5, pady=5)

        self.label_info = tk.Label(parent, text="No broadcast yet", width=35, anchor="w")
        self.label_info.grid(row=row, column=3, padx=5, pady=5)

        self.label_status = tk.Label(parent, text="Not connected", fg="red", width=15)
        self.label_status.grid(row=row, column=5, padx=5, pady=5)

        self.btn_command = tk.Button(parent, text="Send Cmd", command=self.send_test_command, state=tk.DISABLED)
        self.btn_command.grid(row=row, column=6, padx=5, pady=5)

        # Dropdown for mode
        self.mode_var = tk.StringVar(value="Normal")
        self.dropdown_mode = tk.OptionMenu(parent, self.mode_var, *MODE_MAP.keys(), command=self.on_mode_selected)
        self.dropdown_mode.config(state=tk.DISABLED)
        self.dropdown_mode.grid(row=row, column=7, padx=5, pady=5)

        self.label_mode_status = tk.Label(parent, text="Mode: N/A", width=12)
        self.label_mode_status.grid(row=row, column=8, padx=5, pady=5)

        # New label to display heartbeat decode
        self.label_heartbeat_info = tk.Label(parent, text="HB: ---", width=20)
        self.label_heartbeat_info.grid(row=row, column=9, padx=5, pady=5)

    def log(self, msg):
        if self.log_callback:
            self.log_callback(f"[{self.name}] {msg}")

    def set_broadcast_received(self, dev_type, code):
        self.last_broadcast_time = time.time()
        self.indicator.config(bg="green")
        self.broadcast_code = code
        dev_type_str = DEV_TYPE_MAP.get(dev_type, f"Unknown ({dev_type})")
        self.label_info.config(text=f"Code: {code}, Type: {dev_type_str}")

    def update_broadcast_indicator(self, timeout=1.0):
        if time.time() - self.last_broadcast_time > timeout:
            self.indicator.config(bg="red")

    def on_toggle(self):
        if self.conn_var.get() == 1:
            if not self.connected:
                self.log("Toggle switched ON. Initiating connection...")
                self.connect_thread = threading.Thread(target=self.do_connect_request, daemon=True)
                self.connect_thread.start()
            else:
                self.log("Already connected.")
        else:
            if self.connected:
                self.log("Toggle switched OFF. Disconnecting and stopping heartbeat...")
                threading.Thread(target=self.disconnect, args=("User Disconnected",), daemon=True).start()
            else:
                self.log("Already disconnected.")

    def do_connect_request(self):
        try:
            self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.cmd_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.cmd_socket.bind((HOST_IP, CMD_PORT))
            self.log(f"Opened cmd socket on {HOST_IP}:{CMD_PORT}")

            handshake_packet = bytes.fromhex(HANDSHAKE_HEX)
            self.log(f"Sending handshake (fixed): {handshake_packet.hex()}")

            ack_received = False
            for attempt in range(5):
                self.cmd_socket.sendto(handshake_packet, (self.ip, LIDAR_CMD_PORT))
                self.log(f"Handshake attempt {attempt+1} -> {self.ip}:{LIDAR_CMD_PORT}")
                ready = select.select([self.cmd_socket], [], [], 2.0)
                if ready[0]:
                    ack_data, addr = self.cmd_socket.recvfrom(1024)
                    self.log(f"Received ACK: {ack_data.hex()} from {addr}")
                    if len(ack_data) == 16 and ack_data[0:2] == b'\xaa\x01':
                        ret_code = ack_data[10]
                        if ret_code in (0x00, 0x01):
                            self.log(f"Handshake ACK accepted (ret_code=0x{ret_code:02x}).")
                            ack_received = True
                            break
                        else:
                            self.log(f"Unexpected ret_code in handshake ACK: 0x{ret_code:02x}")
                    else:
                        self.log("ACK does not match expected handshake format.")
                else:
                    self.log(f"No ACK received on attempt {attempt+1}.")
                time.sleep(1.0)

            if ack_received:
                self.connected = True
                self.update_conn_status("Connected", "green")
                self.btn_command.config(state=tk.NORMAL)
                self.dropdown_mode.config(state=tk.NORMAL)
                self.open_data_socket()
                self.heartbeat_running = True
                self.heartbeat_thread = threading.Thread(target=self.heartbeat_loop, daemon=True)
                self.heartbeat_thread.start()
            else:
                self.log("Handshake ACK timeout after multiple attempts.")
                self.disconnect("No ACK")
        except Exception as e:
            self.log(f"Handshake error: {e}")
            self.disconnect("Error")
        finally:
            self.update_toggle_state()

    def open_data_socket(self):
        try:
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.data_socket.bind((HOST_IP, DATA_PORT))
            self.log(f"Opened data socket on {HOST_IP}:{DATA_PORT}")
            self.data_running = True
            self.data_thread = threading.Thread(target=self.data_listen_thread, daemon=True)
            self.data_thread.start()
        except Exception as e:
            self.log(f"Failed to open data socket: {e}")

    def data_listen_thread(self):
        self.log("Data listening thread started.")
        while self.data_running:
            try:
                data, addr = self.data_socket.recvfrom(1500)
                self.log(f"Data packet from {addr}, length={len(data)}.")
            except Exception as e:
                self.log(f"Data socket error: {e}")
                break
        self.log("Data listening thread stopped.")

    def heartbeat_loop(self):
        self.log("Starting heartbeat loop.")
        hb_index = 0
        total = len(HEARTBEAT_SEQUENCE)
        while self.heartbeat_running and self.connected:
            try:
                if not self.heartbeat_running:
                    break
                hb_hex = HEARTBEAT_SEQUENCE[hb_index % total]
                heartbeat_packet = bytes.fromhex(hb_hex)
                self.cmd_socket.sendto(heartbeat_packet, (self.ip, LIDAR_CMD_PORT))
                self.log(f"Sent heartbeat ({hb_index+1}): {heartbeat_packet.hex()}")

                if hb_index == 0:
                    # Wait for first heartbeat ACK
                    ready = select.select([self.cmd_socket], [], [], 1.0)
                    if ready[0]:
                        ack_data, addr = self.cmd_socket.recvfrom(1024)
                        self.decode_and_show_heartbeat_ack(ack_data)
                    else:
                        self.log("First heartbeat ACK timeout.")
                        self.disconnect("Heartbeat Timeout")
                        break
                else:
                    # For subsequent heartbeats, do not block. Non-blocking read
                    self.cmd_socket.setblocking(0)
                    try:
                        while True:
                            extra_ack = self.cmd_socket.recv(1024)
                            self.decode_and_show_heartbeat_ack(extra_ack)
                    except socket.error:
                        pass
                    self.cmd_socket.setblocking(1)

                hb_index += 1
                time.sleep(1.0)
            except Exception as e:
                self.log(f"Heartbeat error: {e}")
                self.disconnect("Heartbeat Error")
                break
        self.log("Heartbeat loop stopped.")

    def decode_and_show_heartbeat_ack(self, ack_data: bytes):
        """
        Attempt to parse ack_data as a Heartbeat ACK:
          cmd_set=0x00, cmd_id=0x03
          offset2 => ret_code, offset3 => work_state, offset4 => feature_msg, offset5..8 => ack_msg
        """
        self.log(f"Heartbeat ACK: {ack_data.hex()}")

        if len(ack_data) < 9:
            self.log("Heartbeat ACK too short to parse.")
            return

        cmd_set = ack_data[0]
        cmd_id = ack_data[1]
        if cmd_set == 0x00 and cmd_id == 0x03:
            # ret_code
            ret_code = ack_data[2]
            ret_str = RET_CODE_MAP.get(ret_code, f"0x{ret_code:02x}")

            # work_state
            ws = ack_data[3]
            ws_str = WORK_STATE_MAP.get(ws, f"0x{ws:02x}")

            # feature_msg
            feat = ack_data[4]
            # For demonstration, just show hex
            feat_str = f"0x{feat:02x}"

            # ack_msg
            ack_msg_val = struct.unpack("<I", ack_data[5:9])[0]

            # Build a summary
            summary = f"ret={ret_str}, state={ws_str}, feat={feat_str}, ack_msg=0x{ack_msg_val:08x}"
            self.log(f"Decoded heartbeat: {summary}")

            # Update the label on the GUI row
            self.label_heartbeat_info.config(text=f"HB: {ws_str}")
        else:
            self.log("ACK is not a heartbeat ack (cmd_set/cmd_id mismatch).")

    def on_mode_selected(self, selected_text):
        if not self.connected:
            self.log("Not connected; ignoring mode change.")
            self.label_mode_status.config(text="Mode: Not connected")
            return
        if not self.broadcast_code:
            self.log("No broadcast code stored; cannot set mode.")
            self.label_mode_status.config(text="Mode: No broadcast code")
            return

        if selected_text not in MODE_MAP:
            self.log(f"Unknown mode: {selected_text}")
            return
        mode_val = MODE_MAP[selected_text]
        self.set_lidar_mode(self.broadcast_code, mode_val)
        self.label_mode_status.config(text=f"Mode: {selected_text}")

    def set_lidar_mode(self, bc_code: str, lidar_mode: int):
        """
        LiDAR Command Set approach:
          cmd_set = 0x01
          cmd_id  = 0x00  (Set Mode)
          LiDAR_mode (1 byte)
        """
        self.log(f"Setting LiDAR mode to {lidar_mode} for code {bc_code}")

        # Build data payload: [cmd_set=0x01, cmd_id=0x00, LiDAR_mode(1 byte)]
        data_payload = bytearray()
        data_payload.append(0x01)
        data_payload.append(0x00)
        data_payload.append(lidar_mode & 0xFF)

        preamble = b'\xAA'
        version = b'\x01'
        length_val = len(data_payload)
        length_bytes = length_val.to_bytes(2, 'little')
        cmd_type = b'\x00'
        seq_num = b'\x00\x00'
        header_wo_crc16 = preamble + version + length_bytes + cmd_type + seq_num

        crc16_val = compute_crc16(header_wo_crc16).to_bytes(2, 'little')
        packet_wo_crc32 = header_wo_crc16 + crc16_val + data_payload
        crc32_val = compute_crc32(packet_wo_crc32).to_bytes(4, 'little')
        full_packet = packet_wo_crc32 + crc32_val

        self.cmd_socket.sendto(full_packet, (self.ip, LIDAR_CMD_PORT))
        self.log(f"Sent Set LiDAR Mode (mode={lidar_mode}). Waiting for ACK...")

        ready = select.select([self.cmd_socket], [], [], 2.0)
        if ready[0]:
            ack_data, addr = self.cmd_socket.recvfrom(1024)
            self.log(f"Received ACK for set_lidar_mode: {ack_data.hex()} from {addr}")
        else:
            self.log("No ACK for set_lidar_mode request.")

    def send_test_command(self):
        if not self.connected or not self.cmd_socket:
            self.log("Not connected; can't send command.")
            return
        try:
            cmd_packet = bytes([0x00, 0x04])  # Example
            self.cmd_socket.sendto(cmd_packet, (self.ip, LIDAR_CMD_PORT))
            self.log("Sent Start/Stop sampling command.")
        except Exception as e:
            self.log(f"Error sending command: {e}")

    def disconnect(self, status="Not connected"):
        self.log("Disconnecting...")
        self.connected = False
        self.heartbeat_running = False
        self.data_running = False

        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            try:
                self.heartbeat_thread.join(timeout=0.1)
            except Exception:
                pass
        if self.data_thread and self.data_thread.is_alive():
            try:
                self.data_thread.join(timeout=0.1)
            except Exception:
                pass

        if self.data_socket:
            try:
                self.data_socket.close()
            except Exception:
                pass
            self.data_socket = None
        if self.cmd_socket:
            try:
                self.cmd_socket.close()
            except Exception:
                pass
            self.cmd_socket = None

        self.update_conn_status(status, "red")
        self.btn_command.config(state=tk.DISABLED)
        self.dropdown_mode.config(state=tk.DISABLED)
        self.label_mode_status.config(text="Mode: N/A")
        self.label_heartbeat_info.config(text="HB: ---")
        self.update_toggle_state()

    def update_toggle_state(self):
        new_val = 1 if self.connected else 0
        self.conn_var.set(new_val)

    def update_conn_status(self, text, fg):
        def _update():
            if self.label_status.winfo_exists():
                self.label_status.config(text=text, fg=fg)
        if self.label_status.winfo_exists():
            self.label_status.after(0, _update)

    def close(self):
        self.heartbeat_running = False
        self.data_running = False
        if self.data_socket:
            self.data_socket.close()
            self.data_socket = None
        if self.cmd_socket:
            self.cmd_socket.close()
            self.cmd_socket = None

class LivoxGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Livox Lidar Broadcast & Command Monitor")
        self.master.geometry("1200x450")

        frame = tk.Frame(master)
        frame.pack(padx=10, pady=10, fill=tk.X)

        self.lidars = [
            LidarRow(frame, "Lidar #1", "192.168.1.2", 0, self.log),
            LidarRow(frame, "Lidar #2", "192.168.1.57", 1, self.log),
            LidarRow(frame, "Lidar #3", "192.168.1.65", 2, self.log),
            LidarRow(frame, "Lidar #4", "192.168.1.75", 3, self.log),
        ]

        self.log_area = tk.Text(master, width=140, height=12)
        self.log_area.pack(padx=10, pady=5)

        self.broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.broadcast_socket.bind(("", BROADCAST_PORT))

        self.listening = True
        self.listen_thread = threading.Thread(target=self.listen_for_broadcast, daemon=True)
        self.listen_thread.start()

        self.update_indicators()

    def log(self, msg):
        self.log_area.insert(tk.END, msg + "\n")
        self.log_area.see(tk.END)

    def listen_for_broadcast(self):
        while self.listening:
            try:
                data, addr = self.broadcast_socket.recvfrom(1024)
                ip, port = addr
                info = decode_broadcast(data)
                if info:
                    for lr in self.lidars:
                        if lr.ip == ip:
                            lr.set_broadcast_received(info['dev_type'], info['broadcast_code'])
                            break
            except Exception:
                break

    def update_indicators(self):
        for lr in self.lidars:
            lr.update_broadcast_indicator(timeout=1.0)
        self.master.after(500, self.update_indicators)

    def on_closing(self):
        self.listening = False
        self.broadcast_socket.close()
        for lr in self.lidars:
            lr.close()
        self.master.destroy()

def main():
    root = tk.Tk()
    app = LivoxGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
