import struct
from pydobot import Dobot as OriginalDobot
from pydobot.message import Message

class DobotRail(OriginalDobot):
    """
    擴充版 Dobot 類別，支援滑軌控制
    """

    def set_sliding_rail(self, enable=True):
        """
        發送 SetExtendedLoop 指令 (ID: 3)
        這是控制滑軌的開關，必須先設為 True
        """
        msg = Message()
        msg.id = 3
        msg.ctrl = 0x03  # 寫入並加入隊列
        msg.params = bytearray([0x01 if enable else 0x00])
        print("set_sliding_rail:",msg.params)
        return self._send_command(msg)

    def _set_ptp_cmd(self, x, y, z, r, l, mode, wait=False):
        msg = Message()
        msg.id = 84
        msg.ctrl = 0x03
        
        # 這裡必須是 1 個 Byte + 5 個 Float，總共 21 Bytes
        # 如果你的 params 長度不等於 21，滑軌絕對不會動
        msg.params = struct.pack('<Bfffff', mode, x, y, z, r, l)
        print("params length:",len(msg.params))
        return self._send_command(msg, wait)

    def _set_ptp_cmd_bak(self, x, y, z, r, l=0, mode=1, wait=False):
        """
        覆寫原有的 PTP 指令打包邏輯 (ID: 84)
        將 params 從 1 byte + 4 floats 改為 1 byte + 5 floats
        """
        msg = Message()
        msg.id = 84
        msg.ctrl = 0x03
        # 修改點：'<Bfffff' 增加了一個 f 給 L 軸
        msg.params = struct.pack('<Bfffff', mode, x, y, z, r, l)
        return self._send_command(msg, wait)
    
    def move_to_with_rail(self, x, y, z, r, l=0, wait=False):
        """
        新增的方法，專門用來控制包含滑軌的移動
        """
        return self._set_ptp_cmd(x, y, z, r, l, mode=1, wait=wait)

# --- 使用範例 ---
if __name__ == "__main__":
    # 建立連線 (請修改 COM port)
    device = DobotRail(port='/dev/ttyUSB0', verbose=True)
    
    # 1. 重要：啟動滑軌模式
    device.set_sliding_rail(True)
    
    # 2. 進行移動 (x, y, z, r, l)
    # 假設滑軌移動到 100mm 處
    device.move_to_with_rail(100, 50, 100, 0, l=50, wait=True)
    
    # 3. 關閉連線
    device.close()