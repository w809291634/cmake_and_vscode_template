#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import numpy as np
import cv2
import cvwin
import sys
import time
import copy

#hsv (lowHue,lowSat,lowVal,highHue,highSat,highVal)
color_param={
    "green":(50,94,169,71,200,200),
    "blue":(89,90,176,110,255,255),
    "red":(156,88,187,190,255,255)
}

class Color_Rec(object):
    def __init__(self,color,win=[],win_show=False,winmain_show=True,color_par=None,undistort=True):
        '''
        color：识别颜色
        win：识别窗口：裁剪参数
        win_show：显示所有窗口
        color_par：hsv参数
        undistort：矫正
        '''
        if color_par==None:
            for i in color_param.keys():
                if color==i:
                    self.color_par=color_param[color] 
        else:
            for i in color_par.keys():
                if color==i:
                    self.color_par=color_par[color] 
        self.color=color
        self.win_show=win_show
        self.winmain_show=winmain_show
        self.undistort=undistort
        self.window_name=color+'detection'
        self.lowHue=self.color_par[0]
        self.lowSat=self.color_par[1]
        self.lowVal=self.color_par[2]
        self.highHue=self.color_par[3]
        self.highSat=self.color_par[4]
        self.highVal=self.color_par[5]
        self.win_par=win
        # print(self.window_name)
        self.open_wins=[]
        print(self.color+" color_par",self.color_par)  

    def __win_is_open(self,name):
        for i in self.open_wins:
            if i==name:
                return True
            else:
                return False
        return False

    def open_win(self):
        if self.winmain_show==True and self.__win_is_open(self.window_name)==False:
            cvwin.namedWindow(self.window_name)
            cvwin.createTrackbar('lowHue', self.window_name, self.color_par[0], 255, self.nothing)
            cvwin.createTrackbar('lowSat', self.window_name, self.color_par[1], 255, self.nothing)
            cvwin.createTrackbar('lowVal', self.window_name, self.color_par[2], 255, self.nothing)
            # Higher range colour sliders.
            cvwin.createTrackbar('highHue', self.window_name, self.color_par[3], 255, self.nothing)
            cvwin.createTrackbar('highSat', self.window_name, self.color_par[4], 255, self.nothing)
            cvwin.createTrackbar('highVal', self.window_name, self.color_par[5], 255, self.nothing)
            self.open_wins.append(self.window_name)
            # print("open")

    def nothing(self,*arg):
        pass

    def __undistort(self,src):   #矫正
        DIM=(640, 480)
        K=np.array([[361.6681963247486, 0.0, 331.640979254225], [0.0, 361.1945327740211, 224.49449156302728], [0.0, 0.0, 1.0]])
        D=np.array([[-0.04216543964788291], [0.15543013098889183], [-0.40349493136105163], [0.3373959977368023]])
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        __img = cv2.remap(src, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return __img

    def get_windowname(self):
        return self.window_name

    def __drawpoint(self,img,point):
        point=(point[0],point[1])
        cv2.circle(img,point, 0, (255, 255, 0), 3)   #画点

    def color_det(self,img):
        self.open_win()
        self.img=copy.deepcopy(img)
        if self.undistort==True:
            self.img=self.__undistort(self.img)
        if len(self.win_par)!=0 :
            self.img = self.img[self.win_par[0]:self.win_par[1], self.win_par[2]:self.win_par[3]] #裁剪图像
        # if self.win_show!=False:
        #     cvwin.imshow(self.color+'input_image',self.img)
        
        # Get HSV values from the GUI sliders.
        if self.winmain_show==True:
            self.lowHue = cvwin.getTrackbarPos('lowHue', self.window_name)
            self.lowSat = cvwin.getTrackbarPos('lowSat', self.window_name)
            self.lowVal = cvwin.getTrackbarPos('lowVal', self.window_name)
            self.highHue = cvwin.getTrackbarPos('highHue', self.window_name)
            self.highSat = cvwin.getTrackbarPos('highSat', self.window_name)
            self.highVal = cvwin.getTrackbarPos('highVal', self.window_name)

        # Blur methods available, comment or uncomment to try different blur methods.
        # self.frameBGR = cv2.GaussianBlur(self.img, (7, 7), 0)       #高斯滤波
        # self.frameBGR = cv2.medianBlur(self.img, 7)                     #中值滤波
        # self.frameBGR = cv2.bilateralFilter(self.img, 15 ,75, 75)       #双边滤波
        """kernal = np.ones((15, 15), np.float32)/255
        frameBGR = cv2.filter2D(frameBGR, -1, kernal)"""

        # HSV (Hue, Saturation, Value).
        # Convert the frame to HSV colour model.
        self.hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # HSV values to define a colour range.
        self.colorLow = np.array([self.lowHue,self.lowSat,self.lowVal])
        self.colorHigh = np.array([self.highHue,self.highSat,self.highVal])
        self.mask = cv2.inRange(self.hsv, self.colorLow, self.colorHigh)

        # Show morphological transformation mask
        self.kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, self.kernal)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, self.kernal)
    
        # Put mask over top of the original image.
        self.result = cv2.bitwise_and(self.img, self.img, mask = self.mask)
    
        # Show final output image
        # if self.winmain_show==True:
        #     cvwin.imshow(self.window_name, self.result)   
        
        # if self.win_show!=False:
        #     cvwin.imshow(self.color+'input_image',self.img)
        #     cvwin.imshow(self.color+'blurred', self.frameBGR)
        #     cvwin.imshow(self.color+'mask-plain', self.mask)
        #     cvwin.imshow(self.color+'mask', self.mask)

    def find_pos(self,img):
        self.color_det(img)
        self.canny = cv2.Canny(self.mask, 80, 300, apertureSize = 3)          #检测边缘
        self.contours_img  ,self.contours ,self.hierarchy = cv2.findContours(self.canny ,cv2.RETR_EXTERNAL ,3) #查找轮廓
        # if self.win_show!=False:
            # cvwin.imshow(self.color+"contours_img",self.contours_img)
            # cvwin.imshow(self.color+'input_image',self.img)
            # cvwin.imshow(self.color+'blurred', self.frameBGR)
            # cvwin.imshow(self.color+'mask-plain', self.mask)
            # cvwin.imshow(self.color+'mask', self.mask)

        if len(self.contours)!=0:
            # cv2.drawContours(self.result, self.contours[0], -1, (255, 255, 0), 1)   #画轮廓
            # x, y, w, h = cv2.boundingRect(self.contours[0])
            # cv2.rectangle(self.result, (x, y), (x+w, y+h), (0, 255, 0), 2)          #画无旋转矩形
            rect = cv2.minAreaRect(self.contours[0])      
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(self.result, [box], 0, (0, 255, 255), 1)               #画旋转矩形
            (x, y), radius = cv2.minEnclosingCircle(self.contours[0])               #计算圆
            pos = [int(x), int(y),abs(rect[2])]
            self.__drawpoint(self.img,[pos[0],pos[1]])      
            if self.winmain_show==True:
                cvwin.imshow(self.window_name, self.result)  
            if self.win_show!=False:
                cvwin.imshow(self.color+'image',self.img) 
            return pos
        if self.winmain_show==True:
            cvwin.imshow(self.window_name, self.result)  
        if self.win_show!=False:
            cvwin.imshow(self.color+'image',self.img) 
        return None
           
    def close_win(self):
        if self.__win_is_open(self.window_name)==True:
            if self.win_show!=False:
                # cvwin.destroyWindow(self.color+'input_image')
                # cvwin.destroyWindow(self.color+'blurred')
                # cvwin.destroyWindow(self.color+'mask-plain')
                # cvwin.destroyWindow(self.color+'mask')
                # cvwin.destroyWindow(self.color+'contours_img')
                cvwin.destroyWindow(self.color+'image')

            if self.winmain_show==True:
                cvwin.destroyWindow(self.window_name) 
                self.open_wins.remove(self.window_name)
                # print("close")


if __name__ == '__main__':
    # win=(240,450,145,500)
    # win=[98,420,103,560]
    win=[]
    cap = cv2.VideoCapture(0)   #打开USB摄像头
    if cap.isOpened()!=1:        #视频打开成功
        pass
        # sys.exit()
    # red_det=Color_Rec("red",win,win_show=True,winmain_show=True)
    # green_det=Color_Rec("green",win,win_show=True,winmain_show=True)
    blue_det=Color_Rec("blue",win,win_show=True,winmain_show=True)
    while True:
        ret, img = cap.read()     # 读取一帧
        if ret == False:            # 读取帧失败
            break
        # pos=red_det.find_pos(img)
        # print("red",pos)
        # pos=green_det.find_pos(img)
        # print("green",pos)
        pos=blue_det.find_pos(img)
        print("blue",pos)
