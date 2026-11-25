import cv2
import threading

class ShowImage:
    def __init__(self,cam,planning,detector):
        self.cam=cam
        self._thread = threading.Thread(target=self._show,daemon=True)
        self._thread.start()
        self.pl=planning
        self.dect=detector
        self.image1=None
        self.image2=None
        self.image3=None
        self.image4=None

    def _draw_static(self):
        # 这些没涉及到传进来的img本体
        # cv2.line(self.image1,(0,self.pl.THRESHOLD_Y1),(640,self.pl.THRESHOLD_Y1),(0,0,255),2)
        # cv2.line(self.image1,(0,self.pl.THRESHOLD_Y2),(640,self.pl.THRESHOLD_Y2),(0,0,255),2)
        # cv2.line(self.image1,(0,self.pl.THRESHOLD_Y3),(640,self.pl.THRESHOLD_Y3),(0,0,255),2)
        # cv2.line(self.image1,(0,self.pl.VERIFY_Y1),(640,self.pl.VERIFY_Y1),(0,0,255),2)
        # cv2.line(self.image1,(self.pl.VERIFY_X1,0),(self.pl.VERIFY_X1,480),(0,0,255),2)
        # cv2.line(self.image1,(self.pl.VERIFY_X2,0),(self.pl.VERIFY_X2,480),(0,0,255),2)
        cv2.circle(self.image1,(self.pl.POINT_1[0],self.pl.POINT_1[1]),4,(0,255,0),-1)
        cv2.circle(self.image1,(self.pl.POINT_2[0],self.pl.POINT_2[1]),4,(0,255,0),-1)
        cv2.circle(self.image1,(self.pl.POINT_3[0],self.pl.POINT_3[1]),4,(0,255,0),-1)
        cv2.circle(self.image1,(self.pl.POINT_4[0],self.pl.POINT_4[1]),4,(0,255,0),-1)
        cv2.circle(self.image1,(self.pl.POINT_5[0],self.pl.POINT_5[1]),4,(0,255,0),-1)
        cv2.circle(self.image1,(self.pl.POINT_6[0],self.pl.POINT_6[1]),4,(0,255,0),-1)
        cv2.circle(self.image1,(self.pl.POINT_7[0],self.pl.POINT_7[1]),4,(0,255,0),-1)
    def _draw_detect(self):
        _list=self.dect.final_list
        if _list is None or len(_list) ==0:
            return
        else:
            for target in _list:
                id=target[0]
                cx=target[1]
                cy=target[2]
                w=target[3]
                h=target[4]
                x1 = max(0, int(cx - w / 2) )  # 左上
                y1 = max(0, int(cy - h / 2) )
                x2 = min(640, int(cx + w / 2) )
                y2 = min(480, int(cy + h / 2) )
                cv2.rectangle(self.image2,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(self.image2,id,(x1,int(y1+h)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
    def _show(self):
        try:
            while True:
                if self.cam.img_for_show_img is None:
                    # print("ShowImage类没接收到self.cam.img_for_show_img")
                    continue
                self.image1=self.cam.img_for_show_img.copy()
                # self.image2=self.cam.img_for_show_img.copy()
                # self._draw_static()
                # self._draw_detect()
                if self.image1 is not None:
                    if self.dect.approx is not None and len(self.dect.approx)!=0:
                        for point in self.dect.approx:
                            cv2.circle(self.image1,(point[0],point[1]),3,(0,255,0),-1)
                cv2.imshow("static_threshold",self.image1)
                # cv2.imshow("real_time",self.image2)
                if self.dect.mask is not None:
                    self.image3=self.dect.mask.copy()
                    cv2.imshow("mask",self.image3)

                if self.dect.roi_grey is not None:
                    self.image4=self.dect.roi_grey.copy()
                    cv2.imshow("roi_hsv",self.image4)
                cv2.waitKey(1)
        except KeyboardInterrupt:
            print("show函数结束")
            cv2.destroyAllWindows()
