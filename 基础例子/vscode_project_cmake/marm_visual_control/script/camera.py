#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import cvwin
import cv2
import threading
import time
import numpy as np
import copy
from color_detection import Color_Rec
from plate_detection import Plate_det
from collections import OrderedDict
this = sys.modules[__name__]

this.dir_f = os.path.abspath(os.path.dirname(__file__))
#loc_plate 640*480像素下 定位板框 左上角 和方框像素宽度、高度

this.color_param={
    "green":(50, 102, 175, 71, 200, 200),
    "blue":(89, 174, 150, 110, 255, 255),
    "red":(158, 135, 170, 190, 255, 255)
}

this.bin_param={
    "Binary":[145,300],
}

class AiCamera(object):
    def __init__(self,color,win=[],loc_plate=[141,192,465,386],loc_plate_act=[0.147,0.173,0.092],
    loc_plate_act_origin=[0,0],loc_x_off_mx=25,loc_x_off_mi=9,loc_y_off_mx=15,color_par=None,bin_param=None):
        '''
        win=(240,450,145,500)  (y, y_max, x, x_max)
        loc_plate        单位 pix 640*480像素 定位板框 左上角x,y 和 右下角x,y
        loc_plate_act        单位 m 定位板框实际长度 上底 下底 高[0.147,0.182,0.082]
        loc_x_off_mx         单位 pix 相机倾斜视角X像素偏移 最大值
        loc_y_off_mx         单位 pix 相机倾斜视角y像素偏移 最大值
        loc_plate_origin     单位 pix cv2下像素原点
        loc_plate_act_origin 单位 m  实际定位原点 相对base_link
        ------------- 下底
         -         -
          ---------  上底
        '''
        self.window_name='camera'
        self.win=win
        self.open_wins=[]
        #初始化颜色检测和底板检测
        self.rec_cla_dict=OrderedDict()                                 #颜色识别类,含多个
        if color_par==None:
            for i in color:
                det={
                    "class":Color_Rec(i,win,win_show=False,winmain_show=False,color_par=this.color_param),
                    "pos":None
                }
                self.rec_cla_dict[i]=copy.deepcopy(det)
        else:
            for i in color:
                det={
                    "class":Color_Rec(i,win,win_show=False,winmain_show=False,color_par=color_par),
                    "pos":None
                }
                self.rec_cla_dict[i]=copy.deepcopy(det)
        if bin_param==None:                                              #底板检测类
            self.plate_det=Plate_det(win,win_show=False,winmain_show=False,par=this.bin_param)   
        else:
            self.plate_det=Plate_det(win,win_show=False,winmain_show=False,par=bin_param)   
        #检测摄像头
        cam=self.__camera_check__()
        
        if cam!=-1:
            self.cap = cv2.VideoCapture(cam)
            print("set cam number %d"%cam)
            self.__switch=False
            self.frame=np.array([])
            self.point1=None
            self.point2=None
            self.block=[]
            #定位板参数_实际
            self.loc_plate_act=loc_plate_act
            self.loc_plate_act_origin=loc_plate_act_origin
            #定位板参数_像素
            self.loc_x_off_mx=loc_x_off_mx
            self.loc_x_off_mi=loc_x_off_mi
            self.loc_y_off_mx=loc_y_off_mx
            self.__update_plate_par(loc_plate)

            t = threading.Thread(target=self.__pollcam)
            t.setDaemon(True)
            t.start()
        
    def __camera_check__(self):
        if os.path.exists("/dev/video0"):
            return 0
        if os.path.exists("/dev/video5"):
            return 4
        return -1

    def __undistort(self,src):   #矫正
        DIM=(640, 480)
        K=np.array([[361.6681963247486, 0.0, 331.640979254225], [0.0, 361.1945327740211, 224.49449156302728], [0.0, 0.0, 1.0]])
        D=np.array([[-0.04216543964788291], [0.15543013098889183], [-0.40349493136105163], [0.3373959977368023]])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return img

    def __pollcam(self):
        while True:
            if self.__switch ==True:
                while True:
                    success, frame = self.cap.read()
                    if not success:
                        time.sleep(3)
                        continue
                    img=copy.deepcopy(frame)
                    img=self.__undistort(img)
                    if len(self.win)!=0:
                        img = img[self.win[0]:self.win[1], self.win[2]:self.win[3]] #裁剪图像   
                    if self.point1!=None and self.point2!=None :
                        point1=copy.deepcopy(self.point1)
                        point2=copy.deepcopy(self.point2)
                        self.__drawpoint(img,self.point1)
                        self.__drawpoint(img,self.point2)

                    for i in range(len(self.block)):
                        if self.block[i] !=None:
                            point=[self.block[i][0],self.block[i][1]]
                            self.__drawpoint(img,point)

                    self.__open_win(img)
                    self.frame=frame
                    time.sleep(0.01)
                    if self.__switch !=True:
                        if time.time()-st>5:
                            break
                    else:
                        st=time.time()
            else: 
                self.__close_win()
                time.sleep(1)

    def __win_is_open(self,name):
        for i in self.open_wins:
            if i==name:
                return True
            else:
                return False
        return False

    def __open_win(self,img):
        if self.__win_is_open(self.window_name)==False:    
            #Canny_Threshold
            self.open_wins.append(self.window_name)
        cvwin.imshow("camera",img)

    def __close_win(self):
        if self.__win_is_open(self.window_name)==True:
            cvwin.destroyWindow(self.window_name)
 
    def __update_plate_par(self,loc_plate):
        self.loc_plate=loc_plate
        self.loc_plate_width=self.loc_plate[2]-self.loc_plate[0]
        self.loc_plate_height=self.loc_plate[3]-self.loc_plate[1]
        # if len(self.win)!=0:
        #     self.loc_plate=[loc_plate[0]-self.win[2],loc_plate[1]-self.win[0],loc_plate[2]-self.win[2],loc_plate[3]-self.win[0]]
        self.loc_plate_origin=[self.loc_plate[2]-(self.loc_plate[2]-self.loc_plate[0])/2,self.loc_plate[3]]

    def cam_ctrl(self,switch):
        self.__switch=switch
    
    def __check(self,a,b,err=5,err_a=3.0):
        if type(a)==int and type(b)==int :
            if abs(a-b)<err:
                
                return True
            else:
                return False
        if type(a)==float and type(b)==float :
            if abs(a-b)<err_a:
                return True
            else:
                return False
    
    def __drawpoint(self,img,point):
        point=(point[0],point[1])
        cv2.circle(img,point, 0, (255, 255, 0), 3)   #画点
   
    def block_loc(self):
        # {"red":[pos],"green":pos}
        # {"red":pos,"green":pos}
        self.cam_ctrl(True) 
        self.block=[]  
        self.point1=None
        self.point2=None
        while(np.size(self.frame)==0):
            if np.size(self.frame)!=0:
                break
            time.sleep(0.1)
        #检测底板
        pos=None
        __la_pos=[]
        num=0
        while(pos==None):
            point=self.plate_det.plate_det(self.frame,check=True,err=0.03)       #检测定位板,误差0.03(参数)
            if point!=None:
                break
            time.sleep(0.1)
        while len(__la_pos)<3:
            if len(__la_pos) == 0:
                __la_pos = [point]
                continue
            point=self.plate_det.plate_det(self.frame,check=True,err=0.03)       #检测定位板,误差0.03(参数)
            if point!=None:
                if self.__check(point[0][0],__la_pos[-1][0][0])==True and self.__check(point[0][1],__la_pos[-1][0][1])==True \
                and self.__check(point[1][0],__la_pos[-1][1][0])==True and self.__check(point[1][1],__la_pos[-1][1][1])==True  :
                    __la_pos.append(point)
                else:
                    __la_pos = [point]

        sumx = sumy = sumx1 = sumy1 = 0
        for m in __la_pos:
            sumx += m[0][0]
            sumy += m[0][1]
            sumx1 += m[1][0]
            sumy1 += m[1][1]
        x = sumx / len(__la_pos)
        y = sumy / len(__la_pos)
        x1 = sumx1 / len(__la_pos)
        y1 = sumy1 / len(__la_pos)
        data=[x,y,x1,y1]
        point1=[x,y]
        point2=[x1,y1]
        print "locating plate","--->",data
        self.__update_plate_par(data)
        self.point1=copy.deepcopy(point1)
        self.point2=copy.deepcopy(point2)
            
        #检测颜色物体
        for i in self.rec_cla_dict.keys():
            pos=None
            __la_pos=[]
            num=0
            while(pos==None):
                pos=self.rec_cla_dict[i]["class"].find_pos(self.frame)
                if pos!=None:
                    break
                time.sleep(0.1)

            while len(__la_pos)<5:
                if len(__la_pos) == 0:
                    __la_pos = [pos]
                    continue
                pos=self.rec_cla_dict[i]["class"].find_pos(self.frame)
                if pos!=None:
                    if self.__check(pos[0],__la_pos[-1][0])==True and self.__check(pos[1],__la_pos[-1][1])==True \
                    and self.__check(pos[2],__la_pos[-1][2])==True :
                        __la_pos.append(pos)
                    else:
                        __la_pos = [pos]
                # time.sleep(0.1)

            sumx = sumy = sumw = 0
            for m in __la_pos:
                sumx += m[0]
                sumy += m[1]
                sumw += m[2]
            x = sumx / len(__la_pos)
            y = sumy / len(__la_pos)
            w = sumw / len(__la_pos)
            data=[x,y,w]
            self.rec_cla_dict[i]["pos"]=copy.deepcopy(data)
            self.rec_cla_dict[i]["class"].close_win()
            # print(i,self.rec_cla_dict[i]["pos"])
            self.block.append(data)
        self.cam_ctrl(False)

    def cv2_to_plate(self):
        '''
        CV2坐标系转换到与base_link同方向,单位PIX
                   ^  X
                   |
                   |
             ------------- 下底
              -    |    -
        <---------------  上底
                   ORIGIN
        '''
        # print(self.loc_plate)
        # print("loc_plate_origin:",self.loc_plate_origin)
        self.block_loc()
        for i in self.rec_cla_dict.keys():
            x_proportion=float(self.loc_plate_origin[1]-self.rec_cla_dict[i]['pos'][1]-(self.loc_x_off_mx+self.loc_x_off_mi)/2)/self.loc_plate_height
            x_off=self.loc_x_off_mi+(self.loc_x_off_mx-self.loc_x_off_mi)*x_proportion
            x=self.loc_plate_origin[1]-self.rec_cla_dict[i]['pos'][1]-int(x_off)
            if self.rec_cla_dict[i]['pos'][0]>self.loc_plate_origin[0]:
                y_off=self.rec_cla_dict[i]['pos'][0]-self.loc_plate_origin[0]
                y=-(y_off-self.loc_y_off_mx*(y_off/(self.loc_plate_height/2)))
            else:
                y_off=self.loc_plate_origin[0]-self.rec_cla_dict[i]['pos'][0]
                y=y_off-self.loc_y_off_mx*(y_off/(self.loc_plate_height/2))
            self.rec_cla_dict[i]['pos'][0]=copy.deepcopy(x)
            self.rec_cla_dict[i]['pos'][1]=copy.deepcopy(y)
            # print(i,self.rec_cla_dict[i]["pos"])

    def plate_to_base(self):
        self.cv2_to_plate()
        for i in self.rec_cla_dict.keys():
            # print(self.rec_cla_dict[i]["pos"][0]),self.loc_plate_height
            x_proportion=float(self.rec_cla_dict[i]["pos"][0])/self.loc_plate_height
            y_proportion=float(2*self.rec_cla_dict[i]["pos"][1])/self.loc_plate_width
            x=float(self.loc_plate_act_origin[0])+self.loc_plate_act[2]*x_proportion
            y=float(self.loc_plate_act_origin[1])+(self.loc_plate_act[0]/2+(self.loc_plate_act[1]-self.loc_plate_act[0])/2*x_proportion)*y_proportion
            self.rec_cla_dict[i]["pos"][0]=copy.deepcopy(x)
            self.rec_cla_dict[i]["pos"][1]=copy.deepcopy(y)
            print i,"--->",self.rec_cla_dict[i]["pos"]

if __name__ == '__main__':
    win=[98,420,103,560]
    # win=[]
    aicamer=AiCamera(("red",),win)
    aicamer.cam_ctrl(True)
    # aicamer=AiCamera(("red",),win)
    while True:
        aicamer.plate_to_base()
        time.sleep(1)

