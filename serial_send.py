import serial

class SerialSend:
    def __init__(self,port='/dev/ttyS1', baudrate=115200):
        self.AUTOROTATION_L         = 0X01
        self.AUTOROTATION_R         = 0X02
        self.CAMERA_DOWN            = 0X03
        self.CAMERA_UP              = 0X04
        self.CAMERA_CHECK           = 0X05
        self.GRIPPER_GRASP          = 0X06
        self.GRIPPER_REALSE         = 0X07
        self.FORWARD_FAST           = 0X08
        self.FORWARD_MID            = 0X09
        self.FORWARD_SLOW           = 0X0A
        self.TRACK_LEFT             = 0X0B
        self.TRACK_RIGHT            = 0X0C
        self.BACKWARD_FAST          = 0X0D
        self.BACKWARD_MID           = 0X0E
        self.BACKWARD_SLOW          = 0X0F
        self.GOTO_MID               = 0X11
        self.GOTO_SUPPLIES          = 0X12
        self.GOTO_WOUNDED           = 0X13
        self.TURNTO                 = 0X14
        self.STOP                   = 0X15
        self.DELAY                  = 0X16
        self.START                  = 0X20
        self.PREPARE                = 0X21
        self.PREPARE_DOWN           = 0X22
        self.CAR_RESET              = 0X23
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.5)
            print(f"串口 {port} 打开成功")
        except Exception as e:
            print(f"串口 {port} 打开失败: {e}")
            self.ser = None

    def send(self,data):
        """
        输入要发送的数据（见类定义），直接输出要发送的值
        """
        _data=bytes([data])
        self.ser.write(_data)

    