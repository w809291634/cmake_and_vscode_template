#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import cv2
import numpy as np
import os
import copy
import time
import cvwin
import math
from collections import OrderedDict

c_dir = os.path.split(os.path.realpath(__file__))[0]

def nothing(self,*arg):
    pass

param={
    "Binary":[145,300],
}

list1=[1,2,3,4,5,6,7,8]


if len(ls_dis_mx)>0 and len(point_ls)>0:
    ls_dis_mx.sort(reverse = True)      #最大值排序
    m=init=0
    while len(ls_dis_mx)>0:
        if init!=0 and m<len(ls_dis_mx):
            m+=1
        if m>=len(ls_dis_mx):
            break
        for n in range(len(point_ls)):
            if ls_dis_mx[m]==point_ls[n][4]:
                point1=[point_ls[n][0],point_ls[n][1]]
                point2=[point_ls[n][2],point_ls[n][3]]
                __point1,__point2=self.__pre_point(point1,point2)
                if check==True:
                    if self.slope1!=None and self.slope2!=None:
                        if self.__check_point(__point1,__point2,self.slope1,self.slope2,err)!=True:
                            ls_dis_mx.remove(ls_dis_mx[m])
                            m=0
                            init=0
                            if len(ls_dis_mx)>0:
                                continue
                            else:
                                break
                    else:
                        return -1
                if __point1[0]<__point2[0]:
                    polar=True
                else:
                    polar=False
                __p1=__point1
                __p2=__point2
                point_ls.remove(point_ls[n])
                break
        if  __p1!=None and __p2!=None:
            break