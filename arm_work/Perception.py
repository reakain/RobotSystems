#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import threading
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.yaml_handle as yaml_handle

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


class Perception():
    range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    }

    size = (640, 480)

    x_pid = PID.PID(P=0.1, I=0.00, D=0.008)  # pid初始化
    y_pid = PID.PID(P=0.00001, I=0, D=0)
    z_pid = PID.PID(P=0.005, I=0, D=0)

    def clean_convert_to_colorspace(self, img, colorspace):
        # make a copy of the image, and get the size
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
    
        # resize the image to match our static size selection (handles different camera frame sizes, I guess?)
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        # add fuzzzzz
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        # convert the color space to our specific skew coloring from our lab lights, I think
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    def mask_image_by_color(self, img, color):
        # make a mask of our image that's just that color space range
        frame_mask = cv2.inRange(img,
                                (lab_data[color]['min'][0],
                                lab_data[color]['min'][1],
                                lab_data[color]['min'][2]),
                                (lab_data[color]['max'][0],
                                lab_data[color]['max'][1],
                                lab_data[color]['max'][2]))  #对原图像和掩模进行位运算 


    def get_contour_by_mask(self, mask):
        # I think this is finding any non-enclosed shapes
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        # and then closing them
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        # and then getting the bounds of those shiny new closed shapes
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓

    def get_max_contour(self,contours):
        # get the max contour
        contour_area_temp = 0
        contour_area_max = 0
        areaMaxContour = None

        for c in contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    areaMaxContour = c
        if contour_area_max > 1000:
            return areaMaxContour, contour_area_max  # 返回最大的轮廓
        return


    def get_contour_box(self,contour,img):
        
        # then check if it's a block
        img_h, img_w = img.shape[:2]

        # make a circle around our weird blob shape
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)  # 获取最小外接圆
        center_x = int(Misc.map(center_x, 0, self.size[0], 0, img_w))
        center_y = int(Misc.map(center_y, 0, self.size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, self.size[0], 0, img_w))
        # they don't have any comments, 
        # but I guess if our circle is bigger than 100 radius we just... 
        # go nah, nevermind???
        if radius > 100:
            return img, None
        
        # Make a rectangle around the circle instead
        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))
        cv2.drawContours(img, [box], -1, self.range_rgb[__target_color], 2)

        return img, (center_x, center_y)


    def get_new_movement(self,img,x_dis,y_dis,z_dis,(center_x,center_y),area_max):
        img_h, img_w = img.shape[:2]
        self.x_pid.SetPoint = img_w / 2.0  # 设定
        self.x_pid.update(center_x)  # 当前
        dx = self.x_pid.output
        x_dis += int(dx)  # 输出

        x_dis = 0 if x_dis < 0 else x_dis
        x_dis = 1000 if x_dis > 1000 else x_dis

        self.y_pid.SetPoint = 9000  # 设定
        if abs(area_max - 9000) < 50:
            area_max = 9000
        self.y_pid.update(area_max)  # 当前
        dy = self.y_pid.output
        y_dis += dy  # 输出
        y_dis = 5.00 if y_dis < 5.00 else y_dis
        y_dis = 10.00 if y_dis > 10.00 else y_dis
        
        if abs(center_y - img_h/2.0) < 20:
            self.z_pid.SetPoint = center_y
        else:
            self.z_pid.SetPoint = img_h / 2.0
            
        self.z_pid.update(center_y)
        dy = self.z_pid.output
        z_dis += dy

        z_dis = 32.00 if z_dis > 32.00 else z_dis
        z_dis = 10.00 if z_dis < 10.00 else z_dis

        return x_dis, y_dis, z_dis

