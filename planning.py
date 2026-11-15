import time
import cv2
from serial_send import SerialSend as sd

# format "640*480"
# functions
class Planning():
    def __init__(self,detector):
        self.THRESHOLD_Y1=410
        self.THRESHOLD_Y2=400
        self.VERIFY_Y1=96
        self.THRESHOLD_Y3=390
        self.TIMEOUT=0.1    # TIMEOUT*LOOP=5s
        self.LOOP=50
        self.LOOP_NEAR=10
        self.TIMEOUT_NEAR=0.05
        self.LOOP_VERIFY=10
        self.TIMEOUT_VERIFY=0.01
        self.ACT_INTERVAL=0.2
        # 抓取间隔
        # 松球后退时间
        # 安全区纠偏旋转时间
        self.VERIFY_X1=576
        self.VERIFY_X2=64
        self.POINT_1=[320,120]  # 前
        self.POINT_2=[160,240]  # 左
        self.POINT_3=[320,240]  # 中
        self.POINT_4=[480,240]  # 右
        self.POINT_5=[320,360]  # 后
        self.dect=detector
        self.MY_HOME="blue_home"

        self.state_1=0
        self.state_2=0 

    def find_block_far_important(self,goal_=["black","orange"]):
        _count=0
        sd.send(sd.CAMERA_UP)
        sd.send(sd.GRIPPER_GRASP)
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
                            sd.send(sd.AUTOROTATION_L)
                            continue
                        # 右转
                        elif cx_>330:
                            sd.send(sd.AUTOROTATION_R)
                            continue
                        # 直走
                        else:
                            sd.send(sd.FORWARD_FAST)
                            continue
                    # 很近
                    else:
                        sd.send (sd.CAMERA_DOWN)
                        sd.send (sd.GRIPPER_REALSE)
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
                        sd.send (sd.TRACK_LEFT)
                        continue 
                    # 右转
                    elif cx_>330:
                        sd.send (sd.TRACK_RIGHT)
                        continue 
                    # 直走
                    else:
                        if cy_<=self.THRESHOLD_Y2:
                            sd.send (sd.FORWARD_MID)
                            continue 
                        # 特别近
                        else:
                            sd.send (sd.FORWARD_SLOW)
                            time.sleep(0.5)         # 抓取间隔
                            sd.send (sd.STOP)
                            sd.send (sd.GRIPPER_GRASP)
                            return 1
            else:
                return 0

    def find_block_far_any(self,goal_=["green"]):
        _count=0
        sd.send (sd.CAMERA_UP)
        sd.send (sd.GRIPPER_GRASP)
        while True:
            if _count <self.LOOP:
                # 这个能筛选出goal_列表里包含的最佳目标
                my_list=self.dect.detect_all(goal_) 
                if my_list is None:
                    _count+=1
                    time.sleep(self.TIMEOUT)
                    continue
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    # 比较远
                    if cy_<=self.THRESHOLD_Y1:
                        # 左转
                        if cx_<310:
                            sd.send (sd.AUTOROTATION_L)
                            continue
                        # 右转
                        elif cx_>330:
                            sd.send (sd.AUTOROTATION_R)
                            continue
                        # 直走
                        else:
                            sd.send (sd.FORWARD_FAST)
                            continue
                    # 比较近
                    else:
                        sd.send (sd.CAMERA_DOWN)
                        sd.send (sd.GRIPPER_REALSE)
                        return 1
            else:
                return 0
                

    def find_block_near_any(self,goal_=["green"]):
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
                        sd.send (sd.TRACK_LEFT)
                        continue 
                    # 右转
                    elif cx_>330:
                        sd.send (sd.TRACK_RIGHT)
                        continue 
                    # 直走
                    else:
                        if cy_<=self.THRESHOLD_Y2:
                            sd.send (sd.FORWARD_MID)
                            continue 
                        # 特别近
                        else:
                            sd.send (sd.FORWARD_SLOW)
                            time.sleep(0.5)         # 抓取间隔
                            sd.send (sd.STOP)
                            sd.send (sd.GRIPPER_GRASP)
                            return 1
            else:
                return 0

    def verify_block(self,goal_):# 这个后续可以做一个更加准确的验证函数，即完美贴合夹爪边
        _R=True    
        num_g=0   
        num_b=0
        num_o=0
        _count=0    
        while True:
            if _R==False:
                sd.send (sd.GRIPPER_REALSE)         
                sd.send (sd.BACKWARD_FAST)
                time.sleep(0.5)     # 松球后退时间
                sd.send (sd.STOP)
                sd.send (sd.GRIPPER_GRASP)      # 可修改：将这里闭合夹爪之后进行二次检验
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
                    sd.send (sd.GRIPPER_REALSE)         
                    sd.send (sd.BACKWARD_FAST)
                    time.sleep(0.5)     # 松球后退时间
                    sd.send (sd.STOP)
                    sd.send (sd.GRIPPER_GRASP)      # 可修改：将这里闭合夹爪之后进行二次检验
                    return 0                        # 可修改：每次发现夹错了可以先保持自转，然后跳到近点搜索


    def find_security_far(self,goal_):
        _count=0
        sd.send (sd.AUTOROTATION_L)
        while True:
            if _count<self.LOOP:
                my_list=self.dect.detect_closest(goal_)
                if my_list is None:
                    _count+=1
                    time.sleep(self.TIMEOUT)
                    continue
                else:
                    _count=0
                    _,cx_,cy_,_,_=my_list
                    # 比较远
                    if cy_<=self.THRESHOLD_Y3:
                        # 左转
                        if cx_<310:
                            sd.send (sd.TRACK_LEFT)           
                            continue
                        # 右转
                        elif cx_>330:
                            sd.send (sd.TRACK_RIGHT)    
                            continue
                        # 直走
                        else:
                            sd.send (sd.FORWARD_FAST)
                            continue
                    # 很近
                    else:
                        sd.send (sd.FORWARD_MID)
                        sd.send(sd.CAMERA_DOWN)
                        return 1

    def find_security_near(self,goal_):
        _count=0
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
                    # 在外部
                    if result_1<=0:
                        if result_2<=0 and result_3<=0:
                            sd.send (sd.FORWARD_MID)
                            continue
                        
                        # 特别左偏
                        elif result_2<=0 and result_3>0:
                            sd.send (sd.AUTOROTATION_R)
                            time.sleep(0.5)     # 安全区纠偏旋转时间
                            sd.send(sd.STOP)
                            continue
                        
                        # 特别右偏
                        elif result_2>0 and result_3<=0:
                            sd.send (sd.AUTOROTATION_L)     # 偏太多了，直接大剂量
                            time.sleep(0.5)
                            sd.send(sd.STOP)
                            continue 
                        
                    # 1号点进入
                    elif result_1>0 and result_4<=0:
                        # 直着走进去
                        if result_2<=0 and result_3<=0:
                            sd.send (sd.FORWARD_MID)
                            continue
                        # 左偏
                        elif result_2<=0 and result_3>0:
                            sd.send (sd.TRACK_RIGHT)
                            continue
                        # 右偏
                        elif result_2>0 and result_3<=0:
                            sd.send (sd.TRACK_LEFT)
                            continue
                    # 4号点进入
                    elif result_1>0 and result_4>0:
                        if result_5 <=0:
                            # 直着走进去
                            if result_2<=0 and result_3<=0:
                                sd.send (sd.FORWARD_SLOW)
                                continue
                            
                            # 左偏----# 注意这里没有慢速，看看要不要加上一个
                            elif result_2<=0 and result_3>0:
                                sd.send (sd.TRACK_RIGHT)
                                continue
                            # 右偏
                            elif result_2>0 and result_3<=0:
                                sd.send (sd.TRACK_LEFT)
                                continue
                        # 5号点进入
                        else:
                            sd.send (sd.STOP)
                            sd.send(sd.GRIPPER_REALSE)
                            time.sleep(0.1)
                            sd.send(sd.BACKFORWARD_FAST)
                            time.sleep(0.5)
                            sd.send (sd.STOP)
                            return 1
    def rescue(self):
        """
        该函数为完整规划流程
        """
        if self.state_2<2:
            # 从远处找物料
            if self.state_1 ==0:
                ret_ =self.find_block_far_any()
                # 运行完了
                # if ret_==0:
                #     self.state_1=？？？？？？？？？？？？？？？？
                # elif ret_==1:
                self.state_1=1

                # 从近处找物料
            elif self.state_1==1:
                ret_=self.find_block_near_any()
                if ret_==0:
                    self.state_1=0
                elif ret_==1:
                    self.state_1=2

            # 验证物料
            elif self.state_1==2:
                ret_=self.verify_block(goal_=["green"])
                if ret_==0:
                    self.state_1=0
                elif ret_==1:
                    self.state_1=3

            # 从远处找安全区
            elif self.state_1==3:
                ret_=self.find_security_far(goal_=["home",self.MY_HOME])
                # if ret_==0:             # 要改进，这里留一个超时触发，超时了说明很久没识别到安全区，那么就开到别的地方去看看
                #     self.state_1=???????????????????????/
                # elif ret_==1:
                self.state_1=4

            # 对准放下物块
            elif self.state_1==4:
                ret_=self.find_security_near(goal_=[self.MY_HOME])
                # if ret_==0:
                #     self.state_1=????????
                self.state_2+=1
                self.state_1=0

        # 夹到两个及以上
        elif 2<=self.state_2<4:
            # 从远处找物料
            if self.state_1 ==0:
                ret_ =self.find_block_far_important()
                # 运行完了
                # if ret_==0:
                #     self.state_1=？？？？？？？？？？？？？？？？
                # elif ret_==1:
                self.state_1=1
            
            # 从近处找物料
            elif self.state_1==1:
                ret_=self.find_block_near_important()
                if ret_==0:
                    self.state_1=0
                elif ret_==1:
                    self.state_1=2
            
            # 验证物料
            elif self.state_1==2:
                ret_=self.verify_block(goal_=["green","black","orange"])
                if ret_==0:
                    self.state_1=0
                elif ret_==1:
                    self.state_1=3
            
            # 从远处找安全区
            elif self.state_1==3:
                ret_=self.find_security_far(goal_=["home",self.MY_HOME])
                # if ret_==0:             # 要改进，这里留一个超时触发，超时了说明很久没识别到安全区，那么就开到别的地方去看看
                #     self.state_1=???????????????????????/
                # elif ret_==1:
                self.state_1=4
            
            # 对准放下物块
            elif self.state_1==4:
                ret_=self.find_security_near(goal_=[self.MY_HOME])
                # if ret_==0:
                #     self.state_1=????????
                self.state_2+=1
                self.state_1=0

        else:
            # 从远处找物料
            if self.state_1 ==0:
                ret_ =self.find_block_far_any(goal_=["green","black","orange"])
                # 运行完了
                # if ret_==0:
                #     self.state_1=？？？？？？？？？？？？？？？？
                # elif ret_==1:
                self.state_1=1
            
            # 从近处找物料
            elif self.state_1==1:
                ret_=self.find_block_near_any(goal_=["green","black","orange"])
                if ret_==0:
                    self.state_1=0
                elif ret_==1:
                    self.state_1=2
            
            # 验证物料
            elif self.state_1==2:
                ret_=self.verify_block(goal_=["green","black","orange"])
                if ret_==0:
                    self.state_1=0
                elif ret_==1:
                    self.state_1=3
            
            # 从远处找安全区
            elif self.state_1==3:
                ret_=self.find_security_far(goal_=["home",self.MY_HOME])
                # if ret_==0:             # 要改进，这里留一个超时触发，超时了说明很久没识别到安全区，那么就开到别的地方去看看
                #     self.state_1=???????????????????????/
                # elif ret_==1:
                self.state_1=4
            
            # 对准放下物块
            elif self.state_1==4:
                ret_=self.find_security_near(goal_=[self.MY_HOME])
                # if ret_==0:
                #     self.state_1=????????
                self.state_2+=1
                self.state_1=0



