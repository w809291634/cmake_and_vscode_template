#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
import sys,math,threading
from geometry_msgs.msg import PoseStamped, Pose
import time
import tf
from std_msgs.msg import Int32,Int16
from copy import deepcopy
import math
this = sys.modules[__name__]
import sys   
import signal
from marm_visual_inspection.srv import GenerateSolutions
#机械臂调试
arm_debug=False #True False

def quit(signum, frame):
    print ''
    print 'EXIT APP'
    sys.exit()


class Arm(object):
    def __init__(self,g_open,xarm="varm"):
        self.arm = MoveGroupCommander("manipulator")
        self.xarm=xarm
        if self.xarm=="varm":
            self.gripper = rospy.Publisher('/arm_controller/gripper', Int16, queue_size=0) 
        else:
            self.gripper = rospy.Publisher('/xcar/gripper', Int32, queue_size=0, latch=True)
        self.Solutions_client = rospy.ServiceProxy('/grasp_filter_test/GenerateSolutions', GenerateSolutions)   
        self.arm.set_goal_tolerance(0.001)                   #设置机械臂运动的允许误差值    
        self.arm.set_max_acceleration_scaling_factor(1)      #设置允许的最大速度和加速度
        self.arm.set_max_velocity_scaling_factor(1)

        self.gripper_open=g_open
        self.gripper_close=g_open-80
        # self.all_gohome()
  
        if arm_debug ==True:
            t = threading.Thread(target=self.__arm_tool_pose)
            t.setDaemon(True)
            t.start()

    def __arm_tool_pose(self):
        while True:
            pose=self.getPose()
            print(pose)
            time.sleep(0.1)

    def all_gohome(self,wait=True):
        self.arm.set_named_target('home')                    #控制机械臂先回到初始化位置
        self.arm.go(wait)
        rospy.sleep(1)
        self.setGripper(False)             
        rospy.loginfo("Open the fixture")

    def arm_goHome(self,wait=True):
        self.arm.set_named_target('home')
        self.arm.go(wait)
    
    def arm_goStart(self,wait=True):
        self.arm.set_named_target('start')
        self.arm.go(wait)

    def goPose_rpy(self,a, wait=True, tmout = 8):
        '''
        设置机械臂目的位置, 
        a: 长度为6的数组
            a[0],a[1],a[2], 分别为目的x，y,z 坐标
            a[3],a[4],a[5]， 为机械爪的姿态绕x,y，z轴的旋转
        '''
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"  # group.get_pose_reference_frame()
        #target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = a[0] #x
        target_pose.pose.position.y = a[1] #y
        target_pose.pose.position.z = a[2] #z
        # 自带吸盘的四元数
        q = tf.transformations.quaternion_from_euler(a[3], a[4], a[5]) #RPY

        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(target_pose, self.arm.get_end_effector_link())
        return self.arm.go(True)

    def goPose_qua(self,a, wait=True, tmout = 8):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"  
        #target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = a[0] #x
        target_pose.pose.position.y = a[1] #y
        target_pose.pose.position.z = a[2] #z

        target_pose.pose.orientation.x = a[3]
        target_pose.pose.orientation.y = a[4]
        target_pose.pose.orientation.z = a[5]
        target_pose.pose.orientation.w = a[6]
        self.arm.set_pose_target(target_pose, self.arm.get_end_effector_link())
        return self.arm.go(True)

    def goPosition(self,a):
        self.arm.set_position_target(a, self.arm.get_end_effector_link())
        return self.arm.go(True)


    def getPose(self):
        pose=self.arm.get_current_pose(self.arm.get_end_effector_link())
        pose=[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,
        pose.pose.orientation.w
        ]
        return pose

    def getRpy(self):
        return self.arm.get_current_rpy(arm.get_end_effector_link())

    def get_joints(self):
        return self.arm.get_current_joint_values()

    def set_joint_value_target(self,joint_positions):
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(True)

    def set_arm_joint_value_target(self,joint_positions):
        joint=self.get_joints()
        joint_positions=list(joint_positions)
        joint_positions[4]=joint[4]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go(True)
        
    def setGripper(self,en):
        '''
            机械爪控制en  :True 抓取
                        :False 释放
        '''
        if self.xarm=="varm":
            if en:
                self.gripper.publish(Int16(data=self.gripper_close))
            else:
                self.gripper.publish(Int16(data=self.gripper_open))
        else:
            if en:
                self.gripper.publish(Int32(data=self.gripper_close))
            else:
                self.gripper.publish(Int32(data=self.gripper_open))     
    
    def rotate_gripper(self,angle,offset):  #在当前角度旋转angle度
        angle=math.radians(angle)+offset    #夹具旋转偏移，转换弧度
        joint=self.get_joints()
        joint[4]=joint[4]+angle             #当前位置加上旋转角度
        self.set_joint_value_target(joint)

#拍照位
arm_cam_joint=[0.11535425216048287, -0.616704188215822, 1.40664924293023, 1.8002512210459807, 0.17097973936214012]

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)
    rospy.init_node("arm_debug", log_level=rospy.INFO)
    arm=Arm(-40)                    #定义arm，此时机械臂的夹具打开角度为-40
    while True:
        #获取机械臂当前位姿
        # __pose=arm.getPose()
        # print(__pose)
        # time.sleep(1)
        
        #获取机械臂的当前关节
        joint=arm.get_joints()
        print(joint)
        time.sleep(1)
        
        # arm.rotate_gripper(0)

    



