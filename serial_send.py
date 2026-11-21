import serial
import time
import struct

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
        self.COLLSISION_CHECK       = 0X24
        self.AUTOROTATION_L_SLOW    = 0X25
        self.AUTOROTATION_R_SLOW    = 0X26
        self.CAR_TRACK              = 0X27
        self.CAR_TRACK_SLOW         = 0X28
        self.CAR_AIM                = 0X29
        self.comu_state             = 2
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            print(f"串口 {port} 打开成功")
        except Exception as e:
            print(f"串口 {port} 打开失败: {e}")
            self.ser = None
        self._thread = threading.Thread(target=_receive,daemon=True)
        self._thread.start()

    def _send(self,mode,pixel=0x00):
        """
        输入要发送的数据（见类定义），直接输出要发送的值
        """
        packet = struct.pack('>BBBHBB', 0x30, 0x40, mode, int(pixel), 0xFF,0xFF)
        self.ser.write(packet)
        time.sleep(0.01)

    def _receive(self):
        while True:
            _data=self.ser.read(1)      # 可修改，视情况增加可接受字节数，或者不设置超时，这样可以少一个空的分支
            if _data ==bytes([0x01]):   # 可修改，其他线程也同理，可以先让这些线程暖机启动了 
                self.comu_state=1        # 再启动其他所有函数的权限
            elif _data ==bytes([0xFF]):
                self.comu_state=0
            elif _data ==b"":
                self.comu_state=2  #还没接收到
            else:
                print("出问题了，_receive里面的_data啥也不是")
            return
    
    def send_pro(self,mode,pixel=0x00)
        """
        能够通过应答来验证信息是否被对方成功接收
        """
        _send(self,mode,pixel)
        if self.comu_state==1:
            return
        else: 
        # elif self.comu_state==0 or self.comu_state==2:
            _send(self,mode,pixel)
