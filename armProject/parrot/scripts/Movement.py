#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import rospy
import numpy as np

from kinematics import ik_transform
from warehouse.msg import Grasp
from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

# Make some movement class stuff
class Movement(object):
    
    size = (320, 240)

    def __init__(self, joints_pub):
        #rospy.init_node('movement_node')
        #rospy.loginfo("Arm motion code Init")

        # For motion instructions maybe
        self.x_pid = PID.PID(P=0.06, I=0.005, D=0)  # pid初始化
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.00003, I=0, D=0)

        self.ik = ik_transform.ArmIK()

        self.joints_pub = joints_pub
        
        #y_d = 0
        #roll_angle = 0
        #gripper_rotation = 0
      # 木块对角长度一半
        #square_diagonal = 0.03*math.sin(math.pi/4)
        #F = 1000/240.0
        #adjust_error=False

    # Once we're back at a start point we reset our pids, I think???
    def reset(self):
        # For motion instructions maybe
        self.x_pid = PID.PID(P=0.06, I=0.005, D=0)  # pid初始化
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.00003, I=0, D=0)

    # Do the grabber clacky clacky, can probably use this to drop the cube too lolol
    def clacky_clacky(self):
        # TODO: Have to deal with the joints_pub here too
        bus_servo_control.set_servos(self.joints_pub, 300, ((2, 300),))
        rospy.sleep(0.3)

        bus_servo_control.set_servos(self.joints_pub, 600, ((2, 700),))
        rospy.sleep(0.6)
        
        bus_servo_control.set_servos(self.joints_pub, 600, ((2, 300),))
        rospy.sleep(0.6)
        
        bus_servo_control.set_servos(self.joints_pub, 300, ((2, 500),))
        rospy.sleep(0.3)
        
        bus_servo_control.set_servos(self.joints_pub, 400, ((1, 200),))
        rospy.sleep(0.4)

        bus_servo_control.set_servos(self.joints_pub, 400, ((1, 500),))
        rospy.sleep(0.4)
        
        bus_servo_control.set_servos(self.joints_pub, 400, ((1, 200),))
        rospy.sleep(0.4)
        
        bus_servo_control.set_servos(self.joints_pub, 400, ((1, 500),))
        rospy.sleep(1)

    def center_target(self,img,(center_x,center_y)):
        pass
    
    # Move to grab cube, steal from the sorting "pick" function
    
    def grab_cube(self, grasps, have_adjust=false):
        pass
        #global roll_angle, last_x_dis
        #global adjust, x_dis, y_dis, tag_x_dis, tag_y_dis, adjust_error, gripper_rotation

        #position = grasps.grasp_pos.position
        #rotation = grasps.grasp_pos.rotation
        #approach = grasps.grasp_approach
        #retreat = grasps.grasp_retreat

            # 计算是否能够到达目标位置，如果不能够到达，返回False
        #target1 = ik.setPitchRanges((position.x + approach.x, position.y + approach.y, position.z + approach.z),
        #                                rotation.r, -180, 0)
        #target2 = ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
        #target3 = ik.setPitchRanges((position.x, position.y, position.z + grasps.up), rotation.r, -180, 0)
        #target4 = ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y, position.z + retreat.z),
        #                                rotation.r, -180, 0)

            #if not __isRunning:
                #return False
            #if target1 and target2 and target3 and target4:
                #if not have_adjust:
                    #servo_data = target1[1]
                    #bus_servo_control.set_servos(joints_pub, 1800, (
                    #(3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                    #rospy.sleep(2)
                    #if not __isRunning:
                        #return False

                    # 第三步：移到目标点
                    #servo_data = target2[1]
                    #bus_servo_control.set_servos(joints_pub, 1500, (
                    #(3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                    #rospy.sleep(2)
                    #if not __isRunning:
                        #servo_data = target4[1]
                        #bus_servo_control.set_servos(joints_pub, 1000, (
                        #(1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        #rospy.sleep(1)
                        #return False

                    #roll_angle = target2[2]
                    #gripper_rotation = box_rotation_angle #this uses the max contour stuff 

                    #x_dis = tag_x_dis = last_x_dis = target2[1]['servo6']
                    #y_dis = tag_y_dis = 0


                #else:
                    # 第五步: 对齐
                    #bus_servo_control.set_servos(joints_pub, 500, ((2, 500 + int(F * gripper_rotation)),))
                    #rospy.sleep(0.8)
                    #if not __isRunning:
                        #servo_data = target4[1]
                        #bus_servo_control.set_servos(joints_pub, 1000, (
                        #(1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        #rospy.sleep(1)
                        #return False

                    # 第六步：夹取
                    #bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.grasp_posture - 80),))
                    #rospy.sleep(0.6)
                    #bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.grasp_posture),))
                    #rospy.sleep(0.8)
                    #if not __isRunning:
                        #bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture),))
                        #rospy.sleep(0.5)
                        #servo_data = target4[1]
                        #bus_servo_control.set_servos(joints_pub, 1000, (
                        #(1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        #rospy.sleep(1)
                        #return False

                    # 第七步：抬升物体
                    #if grasps.up != 0:
                        #servo_data = target3[1]
                        #bus_servo_control.set_servos(joints_pub, 500, (
                        #(3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        #rospy.sleep(0.6)
                    #if not __isRunning:
                        #bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture),))
                        #rospy.sleep(0.5)
                        #servo_data = target4[1]
                        #bus_servo_control.set_servos(joints_pub, 1000, (
                        #(1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        #rospy.sleep(1)
                        #return False

                    #return target2[2]
            #else:
                #rospy.loginfo('pick failed')
                #return False
        #grasps=Grasp()
        pass

    # Bring cube to center
    def go_home(self):
        bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))

    # moving to center on cube? or face?
    def get_new_movement(self,img,x_dis,y_dis,z_dis,(center_x,center_y),area_max):
        img_h, img_w = img.shape[:2]
        self.x_pid.SetPoint = img_w / 2.0  # 设定
        self.x_pid.update(center_x)  # 当前
        dx = self.x_pid.output
        x_dis += int(dx)  # 输出

        x_dis = 200 if x_dis < 200 else x_dis
        x_dis = 800 if x_dis > 800 else x_dis

        self.y_pid.SetPoint = 900  # 设定
        if abs(area_max - 900) < 50:
            area_max = 900
        self.y_pid.update(area_max)  # 当前
        dy = self.y_pid.output
        y_dis += dy  # 输出
        y_dis = 0.12 if y_dis < 0.12 else y_dis
        y_dis = 0.25 if y_dis > 0.25 else y_dis
        
        if abs(center_y - img_h/2.0) < 20:
            self.z_pid.SetPoint = center_y
        else:
            self.z_pid.SetPoint = img_h / 2.0
            
        self.z_pid.update(center_y)
        dy = self.z_pid.output
        z_dis += dy

        z_dis = 0.22 if z_dis > 0.22 else z_dis
        z_dis = 0.17 if z_dis < 0.17 else z_dis

        return x_dis, y_dis, z_dis
                 

    def move_with_ik(self,x_dis,y_dis,z_dis):
        target = self.ik.setPitchRanges((0, round(y_dis, 4), round(z_dis, 4)), -90, -85, -95)
        if target:
            servo_data = target[1]
            # TODO: handle this joints pub thing
            bus_servo_control.set_servos(self.joints_pub, 20, (
                (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, x_dis)))
    
if __name__ == '__main__':
    motionControl = Movement()
