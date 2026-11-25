import time
import cv2

# format "640*480"
# functions
class Planning():
    def __init__(self,detector,serial_send):
        self.THRESHOLD_Y1=230
        self.THRESHOLD_Y2=400
        self.VERIFY_Y1=96
        self.THRESHOLD_Y3=230
        self.TIMEOUT=0.02   
        self.LOOP=25
        self.LOOP_NEAR=10
        self.TIMEOUT_NEAR=0.05
        self.LOOP_VERIFY=10
        self.TIMEOUT_VERIFY=0.01
        self.ACT_INTERVAL=0.2
        self.TIMEOUT_SERCURITY=0.2  
        self.LOOP_SERCURITY=100
        # 抓取间隔
        # 松球后退时间
        # 安全区纠偏旋转时间
        self.VERIFY_X1=576
        self.VERIFY_X2=64
        self.POINT_1=[320,140]  # 前
        self.POINT_2=[160,240]  # 左
        self.POINT_3=[320,240]  # 中
        self.POINT_4=[480,240]  # 右
        self.POINT_5=[320,400]  # 后
        self.POINT_6=[160,320]  # 左后
        self.POINT_7=[480,320]  # 右后
        self.dect=detector
        self.MY_HOME="blue_home"
        self.sd=serial_send
        self.state_1=0
        self.state_2=0 
        self.duizhun=False
        self.duizhun_2=False
        self.duizhun3=False

    def find_block_far_important(self,goal_=["black","orange"]):
        _count=0
        self.sd.send(self.sd.CAMERA_UP)
        self.sd.send(self.sd.GRIPPER_GRASP)
        while True:
            if _count<self.LOOP:
                # 这个能筛选出goal_列表里包含的最佳目标
                my_list=self.dect.detect_closest(goal_) 
                if my_list is None:
                    _count+=1
                    time.sleep(self.TIMEOUT)
                    continue
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    if cy_<=self.THRESHOLD_Y1:
                        # 左转
                        if cx_<310: #死区这种我就不单列了
                            self.sd.send(self.sd.AUTOROTATION_L)
                            continue
                        # 右转
                        elif cx_>330:
                            self.sd.send(self.sd.AUTOROTATION_R)
                            continue
                        # 直走
                        else:
                            self.sd.send(self.sd.FORWARD_FAST)
                            continue
                    # 很近
                    else:
                        self.sd.send (self.sd.CAMERA_DOWN)
                        self.sd.send (self.sd.GRIPPER_REALSE)
                        return 1
            else:
                return 0    # 超时


    def find_block_near_important(self,goal_=["black","orange"]):
        _count=0
        while True:
            if _count <self.LOOP_NEAR:
                my_list=self.dect.detect_closest(goal_) 
                if my_list is None:
                    _count+=1
                    time.sleep(self.LOOP_NEAR)
                    continue   
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    # 左转
                    if cx_<310:
                        self.sd.send (self.sd.TRACK_LEFT)
                        continue 
                    # 右转
                    elif cx_>330:
                        self.sd.send (self.sd.TRACK_RIGHT)
                        continue 
                    # 直走
                    else:
                        if cy_<=self.THRESHOLD_Y2:
                            self.sd.send (self.sd.FORWARD_MID)
                            continue 
                        # 特别近
                        else:
                            self.sd.send (self.sd.FORWARD_SLOW)
                            time.sleep(0.5)         # 抓取间隔
                            self.sd.send (self.sd.STOP)
                            self.sd.send (self.sd.GRIPPER_GRASP)
                            return 1
            else:
                return 0

    def find_block_far_any(self,goal_=["green"]):
        self.sd.send (self.sd.AUTOROTATION_L)
        self.sd.send (self.sd.CAMERA_UP)
        self.sd.send (self.sd.GRIPPER_GRASP)
        _count=0
        count_=0
        while True:
            if _count <self.LOOP:
                # 这个能筛选出goal_列表里包含的最佳目标
                my_list=self.dect.detect_closest(goal_) 
                if my_list is None:
                    _count+=1
                    time.sleep(self.TIMEOUT)
                    continue
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    # 比较远
                    if cy_<=self.THRESHOLD_Y1:
                        if self.duizhun==False:
                            # 左快转
                            if cx_<270:
                                self.sd.send (self.sd.AUTOROTATION_L)
                                continue
                            # 右快转
                            elif cx_>370:
                                self.sd.send (self.sd.AUTOROTATION_R)
                                continue
                            # 慢转
                            elif 270<cx_<295 or 345<cx_<370:
                                self.sd.send (self.sd.CAR_AIM,pixel=cx_)
                                continue
                            else:
                                count_+=1
                                if count_ >=20:
                                    self.duizhun=True
                                    count_=0
                                continue
                        # 直走
                        else:
                            print(self.duizhun)
                            self.sd.send (self.sd.FORWARD_FAST,pixel=0 )
                            continue
                    # 比较近
                    else:
                        self.duizhun=False
                        self.sd.send (self.sd.CAMERA_DOWN)
                        self.sd.send (self.sd.GRIPPER_REALSE)
                        self.sd.send(self.sd.STOP)
                        time.sleep(1)
                        return 1
            else:
                return 0

    def test_pid(self):
        self.sd.send (self.sd.AUTOROTATION_L)
        self.sd.send (self.sd.CAMERA_UP)
        self.sd.send (self.sd.GRIPPER_GRASP)
        while True:
            # 这个能筛选出goal_列表里包含的最佳目标
            my_list=self.dect.detect_closest(goal=["green"]) 
            if my_list is None:
                time.sleep(self.TIMEOUT)
                continue
            else:
                _,cx_,cy_,_,_=my_list
                if 310<cx_<330:
                    cx_=320
                # 比较远
                # if cy_<=self.THRESHOLD_Y1:
                self.sd.send(mode=0x27,pixel=int(cx_))
                # # 比较近
                # else:
                #     self.sd.send (self.sd.STOP)
                #     return 1


    def find_block_near_any(self,goal_=["green"]):
        self.sd.send (self.sd.CAMERA_DOWN)
        self.sd.send (self.sd.GRIPPER_REALSE)
        _count=0
        while True:
            if _count <self.LOOP_NEAR:
                my_list=self.dect.detect_closest(goal_) 
                if my_list is None:
                    _count+=1
                    time.sleep(self.LOOP_NEAR)
                    continue   
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    # 转
                    if self.duizhun_2 ==False:
                        if not 310<cx_<330:
                            self.sd.send (self.sd.CAR_AIM,pixel=cx_)
                            continue 
                        # 直走
                        else:
                            self.duizhun_2=True
                    else:
                        if cy_<=self.THRESHOLD_Y2:
                            self.sd.send (self.sd.CAR_TRACK_SLOW,cx_)
                            continue 
                        # 特别近
                        else:
                            self.sd.send (self.sd.FORWARD_SLOW)
                            time.sleep(1)         # 抓取间隔
                            self.sd.send (self.sd.STOP)
                            self.sd.send (self.sd.GRIPPER_GRASP)
                            return 1
            else:
                return 0

    def verify_block(self,goal_):# 这个后续可以做一个更加准确的验证函数，即完美贴合夹爪边
        self.sd.send (self.sd.CAMERA_CHECK)
        time.sleep(0.3)
        _R=True  
        num_g=0   
        num_b=0
        num_o=0
        _count=0    
        while True:
            if _R==False:
                self.sd.send (self.sd.GRIPPER_REALSE)         
                self.sd.send (self.sd.BACKWARD_FAST)
                time.sleep(2)     # 松球后退时间
                self.sd.send (self.sd.STOP)
                self.sd.send (self.sd.GRIPPER_GRASP)      # 可修改：将这里闭合夹爪之后进行二次检验
                return 0                        # 可修改：每次发现夹错了可以先保持自转，然后跳到近点搜索
            else:
                if _count<self.LOOP_VERIFY: 
                    my_list=self.dect.detect_all()  # 解耦合后的函数
                    if my_list is None:
                        _count+=1
                        continue
                    else:
                        _count=0
                        for target in my_list:
                            _id,_x,_y,_,_=target    # _x->cx_   ;_y->cy_
                            if _id not in goal_:
                                if _x>self.VERIFY_X1 or _x<self.VERIFY_X2:
                                    continue
                                else:
                                    if _y>self.VERIFY_Y1:
                                        continue
                                    else:
                                        _R=False
                                        break
                            else:       # goal_里的目标
                                # xy在范围内？
                                if _x>self.VERIFY_X1 or _x<self.VERIFY_X2:
                                    continue
                                else:
                                    if _y>self.VERIFY_Y1:
                                        continue
                                    else:   # 正确目标在范围内
                                        if _id=="green":
                                            num_g+=1
                                        elif _id=="black":
                                            num_b+=1
                                        elif _id=="orange":
                                            num_o+=1
                            if "green" in goal_ and "black" in goal_ and "orange" in goal_:
                                if num_o>=1:
                                    if num_g>0 or num_b>0:
                                        _R=False
                                        break
                                    else:   # 一个橙色都没
                                        if num_g+num_b>3:
                                            _R=False
                                            break
                            elif goal_==["green"]:
                                if num_o >0 or num_b>0:
                                    _R=False
                                    break
                                elif num_g>3:
                                    _R=False
                                    break
                    return 1     # 全部审查合格且不中途退出 
                else: 
                    self.sd.send (self.sd.GRIPPER_REALSE)         
                    self.sd.send (self.sd.BACKWARD_FAST)
                    time.sleep(0.5)     # 松球后退时间
                    self.sd.send (self.sd.STOP)
                    self.sd.send (self.sd.GRIPPER_GRASP)      # 可修改：将这里闭合夹爪之后进行二次检验
                    return 0                        # 可修改：每次发现夹错了可以先保持自转，然后跳到近点搜索


    def find_security_far(self,goal_):
        _count=0
        count_=0
        self.sd.send(self.sd.CAMERA_UP)
        self.sd.send (self.sd.AUTOROTATION_L)
        print(111)
        time.sleep(0.3)
        while True:
            if _count<self.LOOP_SERCURITY:
                my_list=self.dect.detect_closest(goal_)
                # print (my_list)
                if my_list is None:
                    _count+=1
                    time.sleep(self.TIMEOUT_SERCURITY)
                    continue
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    # 比较远
                    if cy_<=self.THRESHOLD_Y3:
                        if self.duizhun3 ==False:
                            # 左转
                            if cx_<270:
                                self.sd.send (self.sd.AUTOROTATION_L)           
                                continue
                            # 右转
                            elif cx_>370:
                                self.sd.send (self.sd.AUTOROTATION_R)    
                                continue
                            # 慢转
                            elif 270<cx_<295 or 345<cx_<370:
                                self.sd.send(self.sd.CAR_AIM,pixel=cx_)
                            # 直走
                            else:
                                count_+=1
                                if count_>=20:
                                    self.duizhun3=True
                                    self.sd.send (self.sd.FORWARD_MID)
                                    continue
                        else:
                            continue
                    # 很近
                    else:
                        self.duizhun3=False
                        return 1
            else:
                self.duizhun3=False
                print("超时")
                self.sd.send(self.sd.STOP)

    def find_security_near(self,goal_):
        # self.sd.send (self.sd.FORWARD_SLOW)
        self.sd.send(self.sd.GRIPPER_GRASP)
        print("闭合")
        self.sd.send(self.sd.CAMERA_DOWN)
        print("从近处寻找安全区")
        _count=0
        _count_1=0
        while True:
            if _count<self.LOOP_NEAR:
                my_list=self.dect.detect_closest(goal_)
                if my_list is None :
                    _count+=1
                    time.sleep(self.TIMEOUT_NEAR)
                else:
                    _,cx_,cy_,w_,h_=my_list
                    # 进行色彩处理将安全区分割出来，获取的是approx角点
                    approx=self.dect.extract_home(cx_,cy_,w_,h_)
                    result_1=cv2.pointPolygonTest(approx,(self.POINT_1[0],self.POINT_1[1]),False)
                    result_2=cv2.pointPolygonTest(approx,(self.POINT_2[0],self.POINT_2[1]),False)
                    result_3=cv2.pointPolygonTest(approx,(self.POINT_3[0],self.POINT_3[1]),False)
                    result_4=cv2.pointPolygonTest(approx,(self.POINT_4[0],self.POINT_4[1]),False)
                    result_5=cv2.pointPolygonTest(approx,(self.POINT_5[0],self.POINT_5[1]),False)
                    result_6=cv2.pointPolygonTest(approx,(self.POINT_6[0],self.POINT_6[1]),False)
                    result_7=cv2.pointPolygonTest(approx,(self.POINT_7[0],self.POINT_7[1]),False)
                    # 在外部
                    if result_1<=0:
                        if result_2<=0 and result_4<=0:
                            self.sd.send (self.sd.FORWARD_SLOW)
                            continue
                        
                        # 特别左偏
                        elif result_2<=0 and result_4>0:
                            self.sd.send (self.sd.AUTOROTATION_R_SLOW)
                            time.sleep(0.5)     # 安全区纠偏旋转时间
                            self.sd.send(self.sd.STOP)
                            continue
                        
                        # 特别右偏
                        elif result_2>0 and result_4<=0:
                            self.sd.send (self.sd.AUTOROTATION_L_SLOW)     # 偏太多了，直接大剂量
                            time.sleep(0.5)
                            self.sd.send(self.sd.STOP)
                            continue 
                        
                    # 1号点进入
                    elif result_1>0 and result_3<=0:
                        # 左偏
                        if result_2<=0 and result_4>0:
                            self.sd.send (self.sd.AUTOROTATION_R_SLOW)
                            time.sleep(0.5)
                            continue
                        # 右偏
                        elif result_2>0 and result_4<=0:
                            self.sd.send (self.sd.AUTOROTATION_L_SLOW)
                            time.sleep(0.5)
                            continue
                        # 直着走进去
                        # if result_2<=0 and result_3<=0:
                        else:
                            self.sd.send (self.sd.FORWARD_SLOW)
                            continue
                        
                    # 3号点进入
                    elif result_1>0 and result_3>0:
                        if result_5 <=0:
                            if _count_1>=5:
                                self.sd.send (self.sd.FORWARD_SLOW)
                                time.sleep(1)
                                _count_1=0
                                continue
                            else:
                                # 左偏----# 注意这里没有慢速，看看要不要加上一个
                                if result_2<=0 and result_4>0:
                                    # self.sd.send (self.sd.TRACK_RIGHT)
                                    self.sd.send (self.sd.AUTOROTATION_R_SLOW)
                                    time.sleep(0.1)
                                    _count_1+=1
                                    continue
                                # 右偏
                                elif result_2>0 and result_4<=0:
                                    self.sd.send (self.sd.AUTOROTATION_L_SLOW)
                                    time.sleep(0.1)
                                    _count_1+=1
                                    continue
                                                            # 直着走进去
                                # if result_2<=0 and result_3<=0:
                                else:
                                    # 左偏
                                    if result_6<=0 and result_7>0:
                                        # self.sd.send (self.sd.TRACK_RIGHT)
                                        self.sd.send (self.sd.AUTOROTATION_R_SLOW)
                                        time.sleep(0.1)
                                        _count_1+=1
                                        continue
                                    # 右偏
                                    elif result_6>0 and result_7<=0:
                                        self.sd.send (self.sd.AUTOROTATION_L_SLOW)
                                        time.sleep(0.1)
                                        _count_1+=1
                                        continue
                                    else:
                                        self.sd.send (self.sd.FORWARD_SLOW)
                                        _count_1=0
                                        continue
                        # 5号点进入
                        else:
                            self.sd.send(self.sd.GRIPPER_REALSE)
                            self.sd.send (self.sd.FORWARD_MID)
                            time.sleep(1.3)
                            self.sd.send(self.sd.BACKWARD_SLOW)
                            time.sleep(2)
                            self.sd.send (self.sd.STOP)
                            self.sd.send(self.sd.GRIPPER_GRASP)
                            return 1
            else:
                self.sd.send(self.sd.STOP)
                return 0

    def rescue(self):
        """
        该函数为完整规划流程
        """
        if self.state_2<2:
        #     # 从远处找物料
        #     if self.state_1 ==0:
        #         print("从远处找物料")
        #         ret_ =self.find_block_far_any()
        #         # 运行完了
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=1

        #         # 从近处找物料
        #     elif self.state_1==1:
        #         print("从近处找物料")
        #         ret_=self.find_block_near_any()
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=2

        #     # 验证物料
        #     elif self.state_1==2:
        #         print("验证物料")
        #         ret_=self.verify_block(goal_=["green"])
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=3

        #     # 从远处找安全区
        #     elif self.state_1==3:
        #         print("从远处找安全区")
        #         ret_=self.find_security_far(goal_=["home",self.MY_HOME])
        #         # if ret_==0:             # 要改进，这里留一个超时触发，超时了说明很久没识别到安全区，那么就开到别的地方去看看
        #         #     self.state_1=???????????????????????/
        #         # elif ret_==1:
        #         self.state_1=4

            # 对准放下物块
            # elif self.state_1==4:
            if 1:
                # print("对准放下物块")
                ret_=self.find_security_near(goal_=[self.MY_HOME])
                # # if ret_==0:
                # #     self.state_1=????????
                # self.state_2+=1
                # self.state_1=0

        # # 夹到两个及以上
        # elif 2<=self.state_2<4:
        #     print("夹到两个及以上")
        #     # 从远处找物料
        #     if self.state_1 ==0:
        #         print("从远处找物料")
        #         ret_ =self.find_block_far_important()
        #         # 运行完了
        #         # if ret_==0:
        #         #     self.state_1=？？？？？？？？？？？？？？？？
        #         # elif ret_==1:
        #         self.state_1=1
            
        #     # 从近处找物料
        #     elif self.state_1==1:
        #         print("从远处找物料")
        #         ret_=self.find_block_near_important()
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=2
            
        #     # 验证物料
        #     elif self.state_1==2:
        #         print("验证物料")
        #         ret_=self.verify_block(goal_=["green","black","orange"])
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=3
            
        #     # 从远处找安全区
        #     elif self.state_1==3:
        #         print("从远处找安全区")
        #         ret_=self.find_security_far(goal_=["home",self.MY_HOME])
        #         # if ret_==0:             # 要改进，这里留一个超时触发，超时了说明很久没识别到安全区，那么就开到别的地方去看看
        #         #     self.state_1=???????????????????????/
        #         # elif ret_==1:
        #         self.state_1=4
            
        #     # 对准放下物块
        #     elif self.state_1==4:
        #         print("对准放下物块")
        #         ret_=self.find_security_near(goal_=[self.MY_HOME])
        #         # if ret_==0:
        #         #     self.state_1=????????
        #         self.state_2+=1
        #         self.state_1=0

        # else:
        #     print("夹到4个及以上")
        #     # 从远处找物料
        #     if self.state_1 ==0:
        #         print("从远处找物料")
        #         ret_ =self.find_block_far_any(goal_=["green","black","orange"])
        #         # 运行完了
        #         # if ret_==0:
        #         #     self.state_1=？？？？？？？？？？？？？？？？
        #         # elif ret_==1:
        #         self.state_1=1
            
        #     # 从近处找物料
        #     elif self.state_1==1:
        #         print("从近处找物料")
        #         ret_=self.find_block_near_any(goal_=["green","black","orange"])
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=2
            
        #     # 验证物料
        #     elif self.state_1==2:
        #         print("验证物料")
        #         ret_=self.verify_block(goal_=["green","black","orange"])
        #         if ret_==0:
        #             self.state_1=0
        #         elif ret_==1:
        #             self.state_1=3
            
        #     # 从远处找安全区
        #     elif self.state_1==3:
        #         print("从远处找安全区")
        #         ret_=self.find_security_far(goal_=["home",self.MY_HOME])
        #         # if ret_==0:             # 要改进，这里留一个超时触发，超时了说明很久没识别到安全区，那么就开到别的地方去看看
        #         #     self.state_1=???????????????????????/
        #         # elif ret_==1:
        #         self.state_1=4
            
        #     # 对准放下物块
        #     elif self.state_1==4:
        #         print("对准放下物块")
        #         ret_=self.find_security_near(goal_=[self.MY_HOME])
        #         # if ret_==0:
        #         #     self.state_1=????????
        #         self.state_2+=1
        #         self.state_1=0


    def test_approx(self):
        list=self.dect.detect_closest(goal=["blue_home","red_home","home"])
        if list is None:
            pass
        else:
            cx=list[1]
            cy=list[2]
            w=list[3]
            h=list[4]
            self.dect.extract_home(cx,cy,w,h)
            


