#test4.py

# 強制測試滑軌 (假設 device 已經連線)
import struct
from pydobot import Dobot
from pydobot.message import Message

device = Dobot(port='/dev/ttyUSB1', verbose=True)

device.clear_alarms()
# 啟用滑軌
msg_enable = Message()
msg_enable.id = 50
msg_enable.ctrl = 0x03
msg_enable.params = struct.pack('<BB', 1, 1)
device._send_command(msg_enable)
device.wait(2)

# 2. 讀取目前的資訊 :value
value = device.getdevicesn()
#print(f"Serial Number: {value}")
device.set_sliding_rail_status( 1 , 0)
#device.go_home() #初始位置
#device.posel() # 讀取滑軌位置 (L)


device._set_ptp_cmd( 0, 0, 0, 0, 400.0)  #滑軌


device.close()
