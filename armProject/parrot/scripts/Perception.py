#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from object_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)



# basicallly a static class, so we don't need to run it as a ros node, because we'll always do this before  a move anyway
class Perception(object):
    
    def __init__(self, size = (320, 240),
                modelFile = "/home/ubuntu/armpi_fpv/src/face_detect/scripts/models/res10_300x300_ssd_iter_140000_fp16.caffemodel",
                configFile = "/home/ubuntu/armpi_fpv/src/face_detect/scripts/models/deploy.prototxt",
                conf_threshold = 0.6,
                range_rgb = {
                            'red': (0, 0, 255),
                            'blue': (255, 0, 0),
                            'green': (0, 255, 0),
                            'black': (0, 0, 0),
                            'white': (255, 255, 255),
                            }):

        # Color range values
        self.range_rgb = range_rgb
        self.color_range = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server
        self.detect_color = ('red', 'green', 'blue')
        # image size setting
        self.size = size

        # Grasp perception
        self.mask1 = cv2.imread('/home/ubuntu/armpi_fpv/src/object_sorting/scripts/mask1.jpg', 0)
        self.mask2 = cv2.imread('/home/ubuntu/armpi_fpv/src/object_sorting/scripts/mask2.jpg', 0)
        self.rows, self.cols = self.mask1.shape

        # Face tracking data and intialization
        self.modelFile = modelFile
        self.configFile = configFile
        self.conf_threshold = conf_threshold
        self.net = cv2.dnn.readNetFromCaffe(self.configFile, self.modelFile)


    #def reset(self):
        # For motion instructions maybe
    #    self.x_pid = PID.PID(P=0.1, I=0.00, D=0.008)  # pid初始化
    #    self.y_pid = PID.PID(P=0.00001, I=0, D=0)
    #    self.z_pid = PID.PID(P=0.005, I=0, D=0)

    # Get the centers of our color cuuube, returns the image, and a tuble of the center of the cube if we found a cube
    def FindColorCube(self,img,target_color_range,start=True):
        #if start = True:
        #    self.reset()
        
        frame_mask = self.clean_build_color_mask(img, target_color_range)
        contours = self.get_contour_by_mask(frame_mask)
        contour,area_max = self.get_max_contour(contours)
        if contour is not None and area_max is not None:
            img_draw,centers = self.get_contour_box(contour,img)
            return img_draw,centers,area_max
        return img, None

        #= self.get_new_movement()

    def FindColorCubeGrab(self,img,target_color_range,start=True):
        frame_mask = self.clean_build_color_mask(img, target_color_range)
        contours = self.get_contour_by_mask(frame_mask)
        contour,area_max = self.get_max_contour(contours)
        if contour is not None and area_max is not None:
            img_draw,centers = self.get_contour_box(contour,img)
            angle = self.get_box_rotation_angle(contour,img)
            centerX,centerY = centers
            if 298 + d_color_map < centerY <= 424 + d_color_map:
                Y = Misc.map(centerY, 298 + d_color_map, 424 + d_color_map, 0.12, 0.12 - 0.04)
            elif 198 + d_color_map < centerY <= 298 + d_color_map:
                Y = Misc.map(centerY, 198 + d_color_map, 298 + d_color_map, 0.12 + 0.04, 0.12)
            elif 114 + d_color_map < centerY <= 198 + d_color_map:
                Y = Misc.map(centerY, 114 + d_color_map, 198 + d_color_map, 0.12 + 0.08, 0.12 + 0.04)
            elif 50 + d_color_map < centerY <= 114 + d_color_map:
                Y = Misc.map(centerY, 50 + d_color_map, 114 + d_color_map, 0.12 + 0.12, 0.12 + 0.08)
            elif 0 + d_color_map < centerY <= 50 + d_color_map:
                Y = Misc.map(centerY, 0 + d_color_map, 50 + d_color_map, 0.12 + 0.16, 0.12 + 0.12)
            else:
                Y = 1

            return img_draw,centers,area_max,angle
        return img, None

    # 获取roi，防止干扰
    def getROI(self,rotation_angle):
        rotate1 = cv2.getRotationMatrix2D((self.rows*0.5, self.cols*0.5), int(rotation_angle), 1)
        rotate_rotate1 = cv2.warpAffine(self.mask2, rotate1, (self.cols, self.rows))
        mask_and = cv2.bitwise_and(rotate_rotate1, self.mask1)
        rotate2 = cv2.getRotationMatrix2D((self.rows*0.5, self.cols*0.5), int(-rotation_angle), 1)
        rotate_rotate2 = cv2.warpAffine(mask_and, rotate2, (self.cols, self.rows))
        frame_resize = cv2.resize(rotate_rotate2, (710, 710), interpolation=cv2.INTER_NEAREST)
        roi = frame_resize[40:280, 184:504]
        
        return roi


    def ColorSortFuck(self,img,target, rotation_angle, d_color_map = 30):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        
        max_area = 0
        color_area_max = None
        areaMaxContour_max = 0
        roi = self.getROI(rotation_angle)
        for i in self.color_range:
            if i in target:
                if i in self.detect_color:
                    target_color_range = self.color_range[i]                
                    frame_mask1 = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 对原图像和掩模进行位运算
                    #mask = cv2.bitwise_and(roi, frame_gray)
                    frame_mask2 = cv2.bitwise_and(roi, frame_mask1)
                    eroded = cv2.erode(frame_mask2, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
                    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
                    #cv2.imshow('mask', dilated)
                    #cv2.waitKey(1)
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                    areaMaxContour, area_max = self.get_max_contour(contours)  # 找出最大轮廓
                    if areaMaxContour is not None:
                        if area_max > max_area and area_max > 100:#找最大面积
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
        if max_area > 100:  # 有找到最大面积
            rect = cv2.minAreaRect(areaMaxContour_max)
            box_rotation_angle = rect[2]
            if box_rotation_angle > 45:
                box_rotation_angle =  box_rotation_angle - 90        
            
            box = np.int0(cv2.boxPoints(rect))   
            for j in range(4): # 映射到原图大小
                box[j, 0] = int(Misc.map(box[j, 0], 0, self.size[0], 0, img_w))
                box[j, 1] = int(Misc.map(box[j, 1], 0, self.size[1], 0, img_h))
            
            cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
            
            centerX = int(Misc.map(((areaMaxContour_max[areaMaxContour_max[:,:,0].argmin()][0])[0] + (areaMaxContour_max[areaMaxContour_max[:,:,0].argmax()][0])[0])/2, 0, self.size[0], 0, img_w))
            centerY = int(Misc.map((areaMaxContour_max[areaMaxContour_max[:,:,1].argmin()][0])[1], 0, self.size[1], 0, img_h))
            
            #cv2.line(img, (0, 430), (640, 430), (0, 255, 255), 2)
            #cv2.circle(img, (int(centerX), int(centerY)), 5, range_rgb[color_area_max], -1)


            return img, (centerX,centerY), max_area, box_rotation_angle
        return img, None, None, None


    #def FindSortColorBox(self,img,target_color_range):

    # Takes an image frame, then finds the person face, and returns the image and 
    # coords of center of face if face is found, otherwise returns None
    def FindFace(self,img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        
        # this chunk makes sure we don't take too many frames
        #if frame_pass:
        #    frame_pass = False
        #    return img
        
        #frame_pass = True

        # Get our new neural net feature blob hell
        blob = cv2.dnn.blobFromImage(img_copy, 1, (150, 150), [104, 117, 123], False, False)
        # Feed it to the neural net
        self.net.setInput(blob)
        # What do your compute eyes seee????
        detections = self.net.forward() #计算识别
        # For each "found" face, let's see what it says
        for i in range(detections.shape[2]):
            # If the confidence for the detection is good enough, then let's get the center of the
            # face box. We're only doing the first one, so god help us if there are multiple people
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                #识别到的人了的各个坐标转换会未缩放前的坐标
                # Get the corners of the box
                x1 = int(detections[0, 0, i, 3] * img_w)
                y1 = int(detections[0, 0, i, 4] * img_h)
                x2 = int(detections[0, 0, i, 5] * img_w)
                y2 = int(detections[0, 0, i, 6] * img_h)             
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2, 8) #将识别到的人脸框出
                # Get the centers
                center_x = int((x1 + x2)/2)
                center_y = int((y1 + y2)/2)
                return img, (center_x,center_y)
                #if action_finish and abs(center_x - img_w/2) < 100:
                #    start_greet = True       
        return img, None

    # build an image mask to only show the color range we specify
    def clean_build_color_mask(self, img,target_color_range):
        # make a copy of the image, and get the size
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
    
        # resize the image to match our static size selection (handles different camera frame sizes, I guess?)
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        # add fuzzzzz
        #frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        # convert the color space to our specific skew coloring from our lab lights, I think
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        # make a mask of our image that's just that color space range
        frame_mask = cv2.inRange(frame_lab,tuple(target_color_range['min']), tuple(target_color_range['max']))  #对原图像和掩模进行位运算 
        return frame_mask

    # Get all the contour blobs after applying a mask to the image
    def get_contour_by_mask(self, mask):
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        # and then getting the bounds of those shiny new closed shapes
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        return contours


    # Get the big boi contour color blob
    def get_max_contour(self,contours):
        # get the max contour
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in self.contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 10:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c
        if contour_area_max > 100:
            return area_max_contour, contour_area_max  # 返回最大的轮廓
        return None, None


    # Get a box around our contour color blob
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

    def get_box_rotation_angle(self,contour,img):
        rect = cv2.minAreaRect(contour)
        box_rotation_angle = rect[2]
        if box_rotation_angle > 45:
            box_rotation_angle =  box_rotation_angle - 90  
            
        return box_rotation_angle
        

    # This we'll put in a movement class instead, because fuck combining this stuff
    # def get_new_movement(self,img,x_pid,y_pid,z_pid,x_dis,y_dis,z_dis,(center_x,center_y),area_max):
    #     img_h, img_w = img.shape[:2]
    #     x_pid.SetPoint = img_w / 2.0  # 设定
    #     x_pid.update(center_x)  # 当前
    #     dx = x_pid.output
    #     x_dis += int(dx)  # 输出

    #     x_dis = 0 if x_dis < 0 else x_dis
    #     x_dis = 1000 if x_dis > 1000 else x_dis

    #     y_pid.SetPoint = 9000  # 设定
    #     if abs(area_max - 9000) < 50:
    #         area_max = 9000
    #     y_pid.update(area_max)  # 当前
    #     dy = y_pid.output
    #     y_dis += dy  # 输出
    #     y_dis = 5.00 if y_dis < 5.00 else y_dis
    #     y_dis = 10.00 if y_dis > 10.00 else y_dis
        
    #     if abs(center_y - img_h/2.0) < 20:
    #         z_pid.SetPoint = center_y
    #     else:
    #         z_pid.SetPoint = img_h / 2.0
            
    #     z_pid.update(center_y)
    #     dy = z_pid.output
    #     z_dis += dy

    #     z_dis = 32.00 if z_dis > 32.00 else z_dis
    #     z_dis = 10.00 if z_dis < 10.00 else z_dis

    #     return x_pid, y_pid, z_pid, x_dis, y_dis, z_dis

