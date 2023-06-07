#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import copy 
import sys   
import signal
import cvwin
import threading
import math
from geometry_msgs.msg import PoseStamped, Pose
this = sys.modules[__name__]

#颜色识别参数
this.color_param={
    "green":(50,94,169,71,200,200),
    "blue":(89,158,161,110,255,255),
    "red":(156,88,187,190,255,255)
}

this.bin_param={
    "Binary":[145,300],
}

#定位板相关参数
loc_plate=[141,192,465,386]         #初始值，机械臂自己查找 单位像素 不用设置
loc_plate_act=[0.153,0.173,0.092]   #定位板实际长度   单位 m 定位板框实际长度 上底 下底 高[0.155,0.173,0.093]\
loc_plate_act_origin=[0.227,-0.0148]        #定位板原点偏移   单位 m
loc_x_off_mx=25                     
loc_x_off_mi=9
loc_y_off_mx=0
loc_opt_par_x=1.0                   #X方向的优化参数
loc_opt_par_y=1.1                   #Y方向的优化参数
'''
loc_plate            单位 pix 640*480像素 定位板框 左上角x,y 和 右下角x,y
loc_plate_act        单位 m 定位板框实际长度 上底 下底 高[0.147,0.182,0.082]
loc_x_off_mx         单位 pix 相机倾斜视角X像素偏移 最大值
loc_y_off_mx         单位 pix 相机倾斜视角y像素偏移 最大值
loc_plate_origin     单位 pix cv2下像素原点
loc_plate_act_origin 单位 m  实际定位原点 相对base_link
'''
#机械臂
arm_g_height=0.1
#拍照位
# arm_cam_pose=[0.1572271238, 0.01332804092, 0.2300313884, 0.04640923861, 0.4895623458, 0.00796998209, 0.87069591217]
arm_cam_joint=[0.11535425216048287, -0.616704188215822, 1.40664924293023, 1.8002512210459807, 0.17097973936214012]
#过渡位
arm_trans_joint=[5.46315397666901e-05, 0.15632741105576997, 1.284452648106851, 0.9141770383249787, -9.986523001948356e-05]
#放料位
arm_place_joint=[-1.5401102540326523, 0.2780434008584921, 1.5998078696019935, 1.0356559126401998, 0.07418549237673772]

def quit(signum, frame):
    print('EXIT APP') 
    sys.exit()

from arm import Arm
from camera import AiCamera

class AiArm(Arm,AiCamera):
    def __init__(self,g_open,color,win=[],loc_plate=[141,192,465,386],loc_plate_act=[0.147,0.173,0.092] ,
    loc_plate_act_origin=[0,0],loc_x_off_mx=25,loc_x_off_mi=9,loc_y_off_mx=15,color_par=None,bin_param=None,):
        super(AiArm,self).__init__(g_open,xarm="xarm")
        super(Arm,self).__init__(color,win,loc_plate,loc_plate_act,loc_plate_act_origin,
        loc_x_off_mx,loc_x_off_mi,loc_y_off_mx,color_par,bin_param)

        self.block_pos=[]
        self.solutions=[]
        t = threading.Thread(target=self.__get_solutions)
        t.setDaemon(True)
        t.start()
    
    def __get_solutions(self):
        while True:
            while len(self.block_pos)>0:
                point=self.block_pos[0]
                Object_pose=Pose()
                Object_pose.position.x=point[1]*loc_opt_par_x
                Object_pose.position.y=point[2]*loc_opt_par_y     
                Object_pose.position.z=arm_g_height
                Object_pose.orientation.x=0
                Object_pose.orientation.y=0
                Object_pose.orientation.z=0
                Object_pose.orientation.w=1
                response = self.Solutions_client(Object_pose)  
                if  len(response.ik_solutions[0].positions)>0:
                    pos=[point[0],response.ik_solutions[1].positions,response.ik_solutions[0].positions,point[3]]
                    self.solutions.append(pos)
                    del self.block_pos[0]
                else:
                    __msg=point[0]+" block is go fail"
                    rospy.logerr(__msg)
                    del self.block_pos[0]
                 
            time.sleep(1) 
        
def aiarmcontrol():
    # win=[98,420,103,560]      #选择剪裁窗口
    win = []
    rospy.init_node("AIARM_NODE", log_level=rospy.INFO)
    aiarm=AiArm(-40,("red","blue",),win,loc_plate,loc_plate_act,loc_plate_act_origin,loc_x_off_mx,loc_x_off_mi,
    loc_y_off_mx,this.color_param,this.bin_param)           #初始化AIarm，注意这里添加的识别物块，打开几个放几个，一直识别
    aiarm.all_gohome()
    aiarm.set_joint_value_target(arm_cam_joint)             #移动到相机位
    time.sleep(2)
    aiarm.plate_to_base()                                   #识别物块的位置
    for i in aiarm.rec_cla_dict.keys():
        point=aiarm.rec_cla_dict[i]["pos"]
        point.insert(0,i)
        aiarm.block_pos.append(point)                       #添加并进行机械臂抓取查询
    while len(aiarm.block_pos)>0 or len(aiarm.solutions)>0:
        while len(aiarm.solutions)>0:
            solution=aiarm.solutions[0]
            # print(solution)
            __msg="Grabbing "+solution[0]+" wood" 
            rospy.loginfo(__msg)
            joint_positions = solution[1]
            aiarm.set_joint_value_target(joint_positions)   #移动到预抓取位
            time.sleep(1)
            if solution[3]>45:                              #物体角度大于45度，反向抓取
                solution[3]=90-solution[3]
                aiarm.rotate_gripper(solution[3],arm_cam_joint[4])
            else:
                aiarm.rotate_gripper(-solution[3],arm_cam_joint[4]) #正向抓取，arm_cam_joint[4]为夹具偏移量
            joint_positions = solution[2]
            aiarm.set_arm_joint_value_target(joint_positions)#移动到抓取位
            time.sleep(1)
            aiarm.setGripper(True)
            time.sleep(1)
            joint=aiarm.get_joints()
            arm_trans_joint[0]=joint[0]         #不旋转1号舵机抬起
            aiarm.set_joint_value_target(arm_trans_joint)
            time.sleep(1)
            aiarm.set_joint_value_target(arm_place_joint)
            time.sleep(1)
            aiarm.setGripper(False) 
            time.sleep(1)
            aiarm.all_gohome() 
            time.sleep(1)
            del aiarm.solutions[0]

        time.sleep(0.1)
    aiarm.all_gohome()
    time.sleep(1)
    print('EXIT APP') 
    sys.exit()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    aiarmcontrol()