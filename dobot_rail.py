import struct
import threading
import time
import serial

# ---------------------------------------------------------
# 1. 訊息處理類別 (修正參數解析)
# ---------------------------------------------------------
class Message:
    def __init__(self, b=None):
        if b is None or b == b'':
            self.id = None
            self.ctrl = None
            self.params = None
            return

        self.id = b[0]
        self.ctrl = b[1]
        self.params = b[2:-1]

    def __repr__(self):
        return "<Message [ID: %s, CTRL: %s, PARAMS: %s]>" % (self.id, self.ctrl, self.params.hex() if self.params else "None")

    def package(self):
        buffer = bytearray([0xAA, 0xAA]) # Header
        if self.params is None:
            buffer.append(2)
        else:
            buffer.append(len(self.params) + 2)
        
        buffer.append(self.id)
        buffer.append(self.ctrl)
        if self.params is not None:
            buffer.extend(self.params)
        
        # Checksum
        checksum = self.id + self.ctrl
        if self.params is not None:
            for byte in self.params:
                checksum += byte
        
        buffer.append((256 - (checksum % 256)) % 256)
        return buffer

# ---------------------------------------------------------
# 2. Dobot 控制類別 (加入滑軌支援)
# ---------------------------------------------------------
class Dobot:
    def __init__(self, port, verbose=False):
        self.verbose = verbose
        self.lock = threading.Lock()
        self.serial = serial.Serial(port, baudrate=115200, timeout=0.1)
        
        # 初始狀態清理
        self._set_queued_cmd_start_execution()
        self._set_queued_cmd_clear()

    def _send_command(self, msg, wait=False):
        with self.lock:
            self.serial.write(msg.package())
            # 這裡簡化了回傳處理，實際應用可加入 index 追蹤
            time.sleep(0.05) 

    def _set_queued_cmd_start_execution(self):
        msg = Message()
        msg.id = 240
        msg.ctrl = 0x01
        return self._send_command(msg)

    def _set_queued_cmd_clear(self):
        msg = Message()
        msg.id = 245
        msg.ctrl = 0x01
        return self._send_command(msg)

    # --- 重要：啟用滑軌功能 ---
    def set_sliding_rail(self, enable=True):
        """啟用或禁用擴充軸 (L軸)"""
        msg = Message()
        msg.id = 3  # SetExtendedLoop
        msg.ctrl = 0x03
        msg.params = bytearray([0x01 if enable else 0x00])
        return self._send_command(msg)

    # --- 重要：修改後的 PTP 指令 (支援 5 軸) ---
    def _set_ptp_cmd(self, x, y, z, r, l, mode, wait=False):
        msg = Message()
        msg.id = 84 # SetPTPCmd
        msg.ctrl = 0x03
        # 打包格式：1 byte (mode) + 5 floats (x, y, z, r, l)
        # '<Bfffff' 代表小端序, 1個unsigned char, 5個float
        msg.params = struct.pack('<Bfffff', mode, x, y, z, r, l)
        return self._send_command(msg, wait)

    def move_to(self, x, y, z, r, l=0, wait=False):
        """移動到指定座標，l 為滑軌位移 (mm)"""
        # 0x01 代表 MOVL_XYZ 模式 (直線移動)
        return self._set_ptp_cmd(x, y, z, r, l, mode=0x01, wait=wait)

    def close(self):
        self.serial.close()

# ---------------------------------------------------------
# 3. 測試執行腳本
# ---------------------------------------------------------
if __name__ == "__main__":
    # 請修改為你的實際 COM Port (Windows: 'COM3', Linux: '/dev/ttyUSB0')
    port_name = '/dev/ttyUSB1' 
    
    device = Dobot(port=port_name)
    print("連接成功")

    try:
        # 第一步：啟用滑軌
        print("正在啟用滑軌...")
        device.set_sliding_rail(True)
        time.sleep(1)

        # 第二步：移動測試
        print("移動手臂與滑軌...")
        # 座標分別為 X, Y, Z, R, L
        device.move_to(x=100, y=0, z=50, r=0, l=100, wait=True)
        time.sleep(2)

        print("僅移動滑軌到 300mm...")
        device.move_to(x=150, y=0, z=50, r=0, wait=True)
        time.sleep(2)

        print("回到滑軌原點...")
        device.move_to(x=200, y=0, y=50, 0, l=0, wait=True)

    except Exception as e:
        print(f"發生錯誤: {e}")
    finally:
        device.close()
        print("連線關閉")
