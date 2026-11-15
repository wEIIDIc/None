from detector import Detector 
from planning import Planning 
from show_img import ShowImage
from cam import Camera


cam=Camera()    # 线程类1
detector=Detector(cam)
planning=Planning(detector) 
show_image=ShowImage(cam,planning,detector)# 线程类2

def main():# 主线程
    planning.rescue()  

if __name__=='__main__':
    main()
