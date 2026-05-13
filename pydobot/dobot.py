import serial
import struct
import time
import threading
import warnings
import logging
from enum import IntEnum
from typing import NamedTuple, Set, Optional

from .message import Message
from .enums import PTPMode
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues

class Alarm(IntEnum):

    COMMON_RESETTING = 0x00,
    COMMON_UNDEFINED_INSTRUCTION = 0x01,
    COMMON_FILE_SYSTEM = 0x02,
    COMMON_MCU_FPGA_COMM = 0x03,
    COMMON_ANGLE_SENSOR = 0x04

    PLAN_INV_SINGULARITY = 0x10,
    PLAN_INV_CALC = 0x11,
    PLAN_INV_LIMIT = 0x12, # !!!
    PLAN_PUSH_DATA_REPEAT = 0x13,
    PLAN_ARC_INPUT_PARAM = 0x14,
    PLAN_JUMP_PARAM = 0x15,
    PLAN_LINE_HAND = 0x16,
    PLAN_LINE_OUT_SPACE = 0x17,
    PLAN_ARC_OUT_SPACE = 0x18,
    PLAN_MOTIONTYPE = 0x19,
    PLAN_SPEED_INPUT_PARAM = 0x1A,
    PLAN_CP_CALC = 0x1B,

    MOVE_INV_SINGULARITY = 0x20,
    MOVE_INV_CALC = 0x21,
    MOVE_INV_LIMIT = 0x22,

    OVERSPEED_AXIS1 = 0x30,
    OVERSPEED_AXIS2 = 0x31,
    OVERSPEED_AXIS3 = 0x32,
    OVERSPEED_AXIS4 = 0x33,

    LIMIT_AXIS1_POS = 0x40,
    LIMIT_AXIS1_NEG = 0x41,
    LIMIT_AXIS2_POS = 0x42,
    LIMIT_AXIS2_NEG = 0x43,
    LIMIT_AXIS3_POS = 0x44,
    LIMIT_AXIS3_NEG = 0x45,
    LIMIT_AXIS4_POS = 0x46,
    LIMIT_AXIS4_NEG = 0x47,
    LIMIT_AXIS23_POS = 0x48,
    LIMIT_AXIS23_NEG = 0x49

    LOSE_STEP_AXIS1 = 0x50,
    LOSE_STEP_AXIS2 = 0x51
    LOSE_STEP_AXIS3 = 0x52
    LOSE_STEP_AXIS4 = 0x53

    OTHER_AXIS1_DRV_ALARM = 0x60,
    OTHER_AXIS1_OVERFLOW = 0x61,
    OTHER_AXIS1_FOLLOW = 0x62,
    OTHER_AXIS2_DRV_ALARM = 0x63,
    OTHER_AXIS2_OVERFLOW = 0x64,
    OTHER_AXIS2_FOLLOW = 0x65,
    OTHER_AXIS3_DRV_ALARM = 0x66,
    OTHER_AXIS3_OVERFLOW = 0x67,
    OTHER_AXIS3_FOLLOW = 0x68,
    OTHER_AXIS4_DRV_ALARM = 0x69,
    OTHER_AXIS4_OVERFLOW = 0x6A,
    OTHER_AXIS4_FOLLOW = 0x6B,

    MOTOR_REAR_ENCODER = 0x70,
    MOTOR_REAR_TEMPERATURE_HIGH = 0x71
    MOTOR_REAR_TEMPERATURE_LOW = 0x72,
    MOTOR_REAR_LOCK_CURRENT = 0x73,
    MOTOR_REAR_BUSV_HIGH = 0x74,
    MOTOR_REAR_BUSV_LOW = 0x75,
    MOTOR_REAR_OVERHEAT = 0x76,
    MOTOR_REAR_RUNAWAY = 0x77,
    MOTOR_REAR_BATTERY_LOW = 0x78,
    MOTOR_REAR_PHASE_SHORT = 0x79,
    MOTOR_REAR_PHASE_WRONG = 0x7A,
    MOTOR_REAR_LOST_SPEED = 0x7B,
    MOTOR_REAR_NOT_STANDARDIZE = 0x7C,
    ENCODER_REAR_NOT_STANDARDIZE = 0x7D,
    MOTOR_REAR_CAN_BROKE = 0x7E,

    MOTOR_FRONT_ENCODER = 0x80,
    MOTOR_FRONT_TEMPERATURE_HIGH = 0x81,
    MOTOR_FRONT_TEMPERATURE_LOW = 0x82,
    MOTOR_FRONT_LOCK_CURRENT = 0x83,
    MOTOR_FRONT_BUSV_HIGH = 0x84,
    MOTOR_FRONT_BUSV_LOW = 0x85,
    MOTOR_FRONT_OVERHEAT = 0x86,
    MOTOR_FRONT_RUNAWAY = 0x87,
    MOTOR_FRONT_BATTERY_LOW = 0x88,
    MOTOR_FRONT_PHASE_SHORT = 0x89,
    MOTOR_FRONT_PHASE_WRONG = 0x8A,
    MOTOR_FRONT_LOST_SPEED = 0x8B,
    MOTOR_FRONT_NOT_STANDARDIZE = 0x8C,
    ENCODER_FRONT_NOT_STANDARDIZE = 0x8D,
    MOTOR_FRONT_CAN_BROKE = 0x8E,

    MOTOR_Z_ENCODER = 0x90,
    MOTOR_Z_TEMPERATURE_HIGH = 0x91,
    MOTOR_Z_TEMPERATURE_LOW = 0x92,
    MOTOR_Z_LOCK_CURRENT = 0x93,
    MOTOR_Z_BUSV_HIGH = 0x94,
    MOTOR_Z_BUSV_LOW = 0x95,
    MOTOR_Z_OVERHEAT = 0x96,
    MOTOR_Z_RUNAWAY = 0x97,
    MOTOR_Z_BATTERY_LOW = 0x98,
    MOTOR_Z_PHASE_SHORT = 0x99,
    MOTOR_Z_PHASE_WRONG = 0x9A,
    MOTOR_Z_LOST_SPEED = 0x9B,
    MOTOR_Z_NOT_STANDARDIZE = 0x9C,
    ENCODER_Z_NOT_STANDARDIZE = 0x9D,
    MOTOR_Z_CAN_BROKE = 0x9E,

    MOTOR_R_ENCODER = 0xA0,
    MOTOR_R_TEMPERATURE_HIGH = 0xA1,
    MOTOR_R_TEMPERATURE_LOW = 0xA2,
    MOTOR_R_LOCK_CURRENT = 0xA3,
    MOTOR_R_BUSV_HIGH = 0xA4,
    MOTOR_R_BUSV_LOW = 0xA5,
    MOTOR_R_OVERHEAT = 0xA6,
    MOTOR_R_RUNAWAY = 0xA7,
    MOTOR_R_BATTERY_LOW = 0xA8,
    MOTOR_R_PHASE_SHORT = 0xA9
    MOTOR_R_PHASE_WRONG = 0xAA,
    MOTOR_R_LOST_SPEED = 0xAB,
    MOTOR_R_NOT_STANDARDIZE = 0xAC,
    ENCODER_R_NOT_STANDARDIZE = 0xAD,
    MOTOR_R_CAN_BROKE = 0xAE,

    MOTOR_ENDIO_IO = 0xB0,
    MOTOR_ENDIO_RS485_WRONG = 0xB1,
    MOTOR_ENDIO_CAN_BROKE = 0xB2

class Dobot:

    def __init__(self, port, verbose=False):
        threading.Thread.__init__(self)
        self.logger = logging.Logger(__name__)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        is_open = self.ser.isOpen()
        if self.verbose:
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')


        self.logger.debug('pydobot: %s open' % self.ser.name if self.ser.isOpen() else 'failed to open serial port')

        self._set_queued_cmd_start_exec()
        self._set_queued_cmd_clear()
        self._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
        self._set_ptp_coordinate_params(velocity=200, acceleration=200)
        self._set_ptp_jump_params(10, 200)
        self._set_ptp_common_params(velocity=100, acceleration=100)

        alarms = self.get_alarms()

        if alarms:
            self.logger.warning(f"Clearing alarms: {', '.join(map(str, alarms))}.")
            self.clear_alarms()
            
        self._get_pose()
    """
        Gets the current command index
    """
    def _get_queued_cmd_current_index(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
        response = self._send_command(msg)
        idx = struct.unpack_from('L', response.params, 0)[0]
        return idx



    def _get_device_sn(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_SET_DEVICE_SN
        response = self._send_command(msg)
        return response

    def _get_device_name(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_SET_DEVICE_NAME
        response = self._send_command(msg)
        return response
    """
        Gets the real-time pose of the Dobot
    """
    def _get_pose(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSE
        response = self._send_command(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]
        try:
            self.l = struct.unpack_from('f', response.params, 32)[0]
        except Exception:
            self.l = 0.0 # 如果沒接滑軌或回傳長度不足，預設為 0

        # if self.verbose:
        #     print("pydobot: x:%03.1f \
        #                     y:%03.1f \
        #                     z:%03.1f \
        #                     r:%03.1f \
        #                     l:%03.1f \
        #                     j1:%03.1f \
        #                     j2:%03.1f \
        #                     j3:%03.1f \
        #                     j4:%03.1f" %
        #           (self.x, self.y, self.z, self.r, self.l, self.j1, self.j2, self.j3, self.j4))
        return response

    def _get_posel(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSEL
        msg.ctrl = ControlValues.ZERO
        msg.params = bytearray([])
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([]))
        response = self._send_command(msg)
        # self.l = struct.unpack_from('f', response.params, 0)[0]

        # if self.verbose:
        print("pydobot: l:%03.1f " %(self.l))
        return response
    
    def _read_message(self):
        time.sleep(0.1)
        b = self.ser.read_all()
        if len(b) > 0:
            msg = Message(b)
            if self.verbose:
                print('pydobot: <<', msg)
            return msg
        return

    def _send_command(self, msg, wait=False):
        self.lock.acquire()
        self._send_message(msg)
        response = self._read_message()
        self.lock.release()

        if not wait:
            return response

        expected_idx = struct.unpack_from('L', response.params, 0)[0]
        if self.verbose:
            print('pydobot: waiting for command', expected_idx)

        while True:
            current_idx = self._get_queued_cmd_current_index()

            if current_idx != expected_idx:
                time.sleep(0.1)
                continue

            if self.verbose:
                print('pydobot: command %d executed' % current_idx)
            break

        return response

    def _send_message(self, msg):
        time.sleep(0.1)
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())

    """
        Executes the CP Command
    """
    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_CP_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    """
        Sets the status of the gripper
    """
    def _set_end_effector_gripper(self, enable=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Sets the status of the suction cup
    """
    def _set_end_effector_suction_cup(self, enable=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Sets the velocity ratio and the acceleration ratio in PTP mode
    """
    def _set_ptp_joint_params(self, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', v_x)))
        msg.params.extend(bytearray(struct.pack('f', v_y)))
        msg.params.extend(bytearray(struct.pack('f', v_z)))
        msg.params.extend(bytearray(struct.pack('f', v_r)))
        msg.params.extend(bytearray(struct.pack('f', a_x)))
        msg.params.extend(bytearray(struct.pack('f', a_y)))
        msg.params.extend(bytearray(struct.pack('f', a_z)))
        msg.params.extend(bytearray(struct.pack('f', a_r)))
        return self._send_command(msg)

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """
    def _set_ptp_coordinate_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """
    def _set_ptp_jump_params(self, jump, limit):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', jump)))
        msg.params.extend(bytearray(struct.pack('f', limit)))
        return self._send_command(msg)


    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """
    def _set_ptp_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
        Executes PTP command
    """

    def _set_ptp_cmd(self, x, y, z, r, l, mode=PTPMode.MOVL_ANGLE, wait=False):  #MOVL_INC
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_SLIDING_RAIL_CMD   #SET_PTP_CMD
        msg.ctrl = ControlValues.THREE
        
        # 這裡必須是 1 個 Byte + 5 個 Float，總共 21 Bytes
        # 如果你的 params 長度不等於 21，滑軌絕對不會動
        #print(f"DEBUG: mode={int(mode.value)} ")
        #print(f"DEBUG: x={x}, y={y}, z={z}, r={r}, l={l}")
        msg.params = struct.pack('<Bfffff',
                        int(mode.value),
                        float(x),
                        float(y),
                        float(z),
                        float(r),
                        float(l)
                    )
        #print("params length:",len(msg.params))
        return self._send_command(msg, wait)

    def _set_ptp_cmd_bak(self, x, y, z, r, l=0, mode=PTPMode.MOVJ_XYZ_INC, wait=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params = struct.pack('<Bfffff', 
                             int(mode.value), 
                             float(x), 
                             float(y), 
                             float(z), 
                             float(r), 
                             float(l))
        return self._send_command(msg, wait)

    def _set_ptp_rail_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_SLIDING_RAIL_COMMON_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)
    
    def _set_ptp_rail_cmd(self, x, y, z, r, l, mode, wait=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_SLIDING_RAIL_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode.value]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        msg.params.extend(bytearray(struct.pack('f', l)))
        return self._send_command(msg, wait)

    def _set_homing(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_HOME_CMD
        msg.ctrl = ControlValues.TWO
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        msg.params.extend(bytearray([0x01]))
        msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)



    """
        Clears command queue
    """
    def _set_queued_cmd_clear(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    """
        Start command
    """
    def _set_queued_cmd_start_exec(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    """
        Wait command
    """
    def _set_wait_cmd(self, ms):
        msg = Message()
        msg.id = 110
        msg.ctrl = 0x03
        msg.params = bytearray(struct.pack('I', ms))
        return self._send_command(msg)

    """
        Stop command
    """
    def _set_queued_cmd_stop_exec(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    def _get_eio_level(self, address):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_EIO
        msg.ctrl = ControlValues.ZERO
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        return self._send_command(msg)

    def _set_eio_level(self, address, level):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_EIO
        msg.ctrl = ControlValues.ONE
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        msg.params.extend(bytearray([level]))
        return self._send_command(msg)

    def get_eio(self, addr):
        return self._get_eio_level(addr)

    def set_eio(self, addr, val):
        return self._set_eio_level(addr, val)

    def close(self):
        self._on = False
        self.lock.acquire()
        self.ser.close()
        if self.verbose:
            print('pydobot: %s closed' % self.ser.name)
        self.lock.release()

    def go(self, x, y, z, r=0.):
        warnings.warn('go() is deprecated, use move_to() instead')
        self.move_to(x, y, z, r)

    def move_to(self, x, y, z, r, l=0, wait=False):
        # self._set_ptp_cmd(x, y, z, r, l, mode=PTPMode.MOVL_XYZ, wait=wait)
        self._set_ptp_cmd(x, y, z, r, l, mode=2, wait=wait)

    def suck(self, enable):
        self._set_end_effector_suction_cup(enable)

    def grip(self, enable):
        self._set_end_effector_gripper(enable)

    def speed(self, velocity=100., acceleration=100.):
        self._set_ptp_common_params(velocity, acceleration)
        self._set_ptp_coordinate_params(velocity, acceleration)

    def wait(self, ms):
        self._set_wait_cmd(ms)

    def getdevicesn(self):
        response = self._get_device_sn()
        return response

    def getdevicename(self):
        response = self._get_device_name()
        return response

    def pose(self):
        response = self._get_pose()
        #print("RAW HEX_pose:", response.params.hex())
        x = struct.unpack_from('f', response.params, 0)[0]
        y = struct.unpack_from('f', response.params, 4)[0]
        z = struct.unpack_from('f', response.params, 8)[0]
        r = struct.unpack_from('f', response.params, 12)[0]
        j1 = struct.unpack_from('f', response.params, 16)[0]
        j2 = struct.unpack_from('f', response.params, 20)[0]
        j3 = struct.unpack_from('f', response.params, 24)[0]
        j4 = struct.unpack_from('f', response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4
    
    def posel(self):
        response = self._get_posel()
        print("RAW HEX_posel:", response.params.hex())
        print("LEN:", len(response.params))
        l = struct.unpack_from('f', response.params, 0)[0]
        return l


    def get_alarms(self) -> Set[Alarm]:
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_ALARMS_STATE
        response = self._send_command(msg)  # 32 bytes

        ret: Set[Alarm] = set()

        for idx in range(16):
            alarm_byte = struct.unpack_from('B', response.params, idx)[0]
            for alarm_index in [i for i in range(alarm_byte.bit_length()) if alarm_byte & (1 << i)]:
                ret.add(Alarm(idx*8+alarm_index))
        return ret

    def clear_alarms(self) -> None:
        msg = Message()
        msg.id = 20
        msg.ctrl = 0x01
        self._send_command(msg)  # empty response
            
    def set_sliding_rail(self, enable=True):
        # ID 3 指令通常用於設定擴充軸 (Extended Loop)
        msg = Message()
        msg.id = 3 
        msg.ctrl = 0x03
        msg.params = bytearray([0x01 if enable else 0x00])
        return self._send_command(msg)


    def go_home(self):
        response = self._set_homing()
        return response



    def get_device_version(self):
        # request = Message([0xAA, 0xAA], 2, 2, False, False, [], direction='out')
        msg = Message()
        msg.id = 2 
        msg.ctrl = 0x02
        msg.params = bytearray([])
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([]))
        # return self.send(request)
        return self._send_command(msg)


    def set_sliding_rail_status(self, enable, version):
        # request = Message([0xAA, 0xAA], 2, 3, True, False, [], direction='out')
        msg = Message()
        msg.id = 50
        msg.ctrl = 0x03
        #msg.params = bytearray([])
        # msg.params.extend(bytearray([0x01]))
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([]))

        msg.params = struct.pack('<BB', int(enable), int(version))
        return self._send_command(msg)

    # Time in milliseconds since start
    def get_device_time(self):
        request = Message([0xAA, 0xAA], 2, 4, False, False, [], direction='out')
        return self.send(request)

    def get_device_id(self):
        request = Message([0xAA, 0xAA], 2, 5, False, False, [], direction='out')
        return self.send(request)
    def get_sliding_rail_pose(self):
        # request = Message([0xAA, 0xAA], 2, 13, False, False, [], direction='out')
        msg = Message()
        # msg.id = 13 
        # msg.ctrl = 0x02
        
        msg.id = CommunicationProtocolIDs.GET_POSE
        msg.ctrl = ControlValues.TWO
        msg.params = bytearray([])
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([0x00]))
        # msg.params.extend(bytearray([]))
        return self._send_command(msg)   
    
    def get_sliding_rail_posel(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSEL
        msg.ctrl = ControlValues.ZERO
        msg.params = bytearray([])

        response = self._send_command(msg)
        #print("RAW params hex:", response.params.hex())

        return self._send_command(msg)
    
    
    def speedl(self, velocity=100., acceleration=100.):
        self._set_ptp_rail_common_params(velocity, acceleration)
        self._set_ptp_coordinate_params(velocity, acceleration)
        
    def movel_to(self, x, y, z, r, l=0, wait=False):
        self._set_ptp_rail_cmd(x, y, z, r, l, mode=PTPMode.MOVJ_XYZ_INC, wait=wait)
        # self._set_ptp_cmd(x, y, z, r, l, mode=2, wait=wait)
    

