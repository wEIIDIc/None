import cv2
import threading
import numpy as np
import time
######要写一个释放线程的函数stop
class Camera:
    def __init__(self):
        self.video = cv2.VideoCapture(0)    #"D:\CodesProject\GC-Intelligent-Rescue\videos\train_8.avi"
        if not self.video.isOpened():
            print("无法打开摄像头")
            exit(1)
        # 设置 MJPG 格式和分辨率
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 关闭自动白平衡
        self.video.set(cv2.CAP_PROP_AUTO_WB, 0)
        # 关闭自动曝光
        self.video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        # 固定曝光值
        self.video.set(cv2.CAP_PROP_EXPOSURE, 100)
        # 设置帧率
        fps = 60
        self.video.set(cv2.CAP_PROP_FPS, fps)
        self.img_for_show_img=None
        self.img_for_detector=None
        self.thread=threading.Thread(target=self._get_frame,daemon=True)
        self.thread.start()

    def _get_frame(self):
        while True:
            ret,frame=self.video.read()
            if not ret :
                # print("114")
                continue
            # frame = np.zeros((480, 640, 3), dtype=np.uint8)
            self.img_for_show_img=frame.copy()
            self.img_for_detector=frame.copy()
            # time.sleep(1)
    
