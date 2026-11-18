from detector import Detector 
from planning import Planning 
from show_img import ShowImage
from cam import Camera
from serial_send import SerialSend
import time 

serial_send=SerialSend()
cam=Camera()    # 线程类1
detector=Detector(cam)
planning=Planning(detector,serial_send) 
show_image=ShowImage(cam,planning,detector)# 线程类2

def main():# 主线程
    time.sleep(3)
    while True:
        # planning.rescue()
        planning.test_split()
    print("主线程已退出")

if __name__=='__main__':
    main()
