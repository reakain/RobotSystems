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
    
    d_pulse = 5
    Z_DIS = 0.2
    size = (320, 240)

    def __init__(self, joints_pub):
        #rospy.init_node('movement_node')
        #rospy.loginfo("Arm motion code Init")

        self.x_dis = 500
        self.last_x_dis = self.x_dis
        self.y_dis = 0.167
        self.last_y_dis = self.y_dis
        self.z_dis = self.Z_DIS
        # For motion instructions maybe
        self.x_pid = PID.PID(P=0.06, I=0.005, D=0)  # pid初始化
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.00003, I=0, D=0)

        self.ik = ik_transform.ArmIK()

        self.joints_pub = joints_pub
        
        self.y_d = 0
        self.grasps=Grasp()
        roll_angle = 0
        gripper_rotation = 0
      # 木块对角长度一半
        square_diagonal = 0.03*math.sin(math.pi/4)
        F = 1000/240.0
        adjust_error=False
        last_x_dis=x_dis

    # Once we're back at a start point we reset our pids, I think???
    def reset(self):
        self.x_dis = 500
        self.last_x_dis = self.x_dis
        self.y_dis = 0.167
        self.last_y_dis = self.y_dis
        self.z_dis = self.Z_DIS
        # For motion instructions maybe
        self.x_pid.clear()
        self.y_pid.clear()
        self.z_pid.clear()

    # Do the grabber clacky clacky, can probably use this to drop the cube too lolol
    def clacky_clacky(self):
        bus_servo_control.set_servos(self.joints_pub, 300, ((1, 100),(2, 300),))
        rospy.sleep(0.3)

        bus_servo_control.set_servos(self.joints_pub, 600, ((1, 500),(2, 700),))
        rospy.sleep(0.6)
        
        bus_servo_control.set_servos(self.joints_pub, 600, ((1, 100),(2, 300),))
        rospy.sleep(0.6)
        
        bus_servo_control.set_servos(self.joints_pub, 300, ((1, 500),(2, 500),))
        # rospy.sleep(0.3)
        
        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 100),))
        # rospy.sleep(0.4)

        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 500),))
        # rospy.sleep(0.4)
        
        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 100),))
        # rospy.sleep(0.4)
        
        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 500),))
        rospy.sleep(1)

    
    def approach_cube(self):
        # 夹取的位置
        self.grasps.grasp_pos.position.x = X
        self.grasps.grasp_pos.position.y = Y
        self.grasps.grasp_pos.position.z = Misc.map(Y - 0.15, 0, 0.15, color_z_min, color_z_max)
        # 夹取时的俯仰角
        self.grasps.grasp_pos.rotation.r = -175
        
        # 夹取后抬升的距离
        self.grasps.up = 0
        
        # 夹取时靠近的方向和距离
        self.grasps.grasp_approach.y = -0.01
        self.grasps.grasp_approach.z = 0.02
        
        # 夹取后后撤的方向和距离
        self.grasps.grasp_retreat.z = 0.04
        
        # 夹取前后夹持器的开合
        self.grasps.grasp_posture = 450
        self.grasps.pre_grasp_posture = 75

        result = self.grab_cube()
        return result


    # Move to grab cube, steal from the sorting "pick" function   
    def grab_cube(self, have_adjust=False):
        global roll_angle
        global adjust, adjust_error, gripper_rotation

        position = self.grasps.grasp_pos.position
        rotation = self.grasps.grasp_pos.rotation
        approach = self.grasps.grasp_approach
        retreat = self.grasps.grasp_retreat
        y_d = 0
        roll_angle = 0
        square_diagonal = 0.03*math.sin(math.pi/4)
        F = 1000/240.0
        adjust_error=False
        self.last_x_dis=self.x_dis

            # 计算是否能够到达目标位置，如果不能够到达，返回False
        target1 = self.ik.setPitchRanges((position.x + approach.x, position.y + approach.y, position.z + approach.z),
                                        rotation.r, -180, 0)
        target2 = self.ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
        target3 = self.ik.setPitchRanges((position.x, position.y, position.z + self.grasps.up), rotation.r, -180, 0)
        target4 = self.ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y, position.z + retreat.z),
                                        rotation.r, -180, 0)

            if target1 and target2 and target3 and target4:
                if not have_adjust:
                    servo_data = target1[1]
                    bus_servo_control.set_servos(self.joints_pub, 1800, (
                    (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                    rospy.sleep(2)

                    # 第三步：移到目标点
                    servo_data = target2[1]
                    bus_servo_control.set_servos(self.joints_pub, 1500, (
                    (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                    rospy.sleep(2)


                    roll_angle = target2[2]
                    gripper_rotation = 0 #box_rotation_angle #this uses the max contour stuff 

                    self.x_dis = last_x_dis = target2[1]['servo6']
                    self.y_dis  = 0


                else:
                    # 第五步: 对齐
                    bus_servo_control.set_servos(self.joints_pub, 500, ((2, 500 + int(F * gripper_rotation)),))
                    rospy.sleep(0.8)

                    # 第六步：夹取
                    bus_servo_control.set_servos(self.joints_pub, 500, ((1, self.grasps.grasp_posture - 80),))
                    rospy.sleep(0.6)
                    bus_servo_control.set_servos(self.joints_pub, 500, ((1, self.grasps.grasp_posture),))
                    rospy.sleep(0.8)

                    # 第七步：抬升物体
                    if self.grasps.up != 0:
                        servo_data = target3[1]
                        bus_servo_control.set_servos(self.joints_pub, 500, (
                        (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        rospy.sleep(0.6)

                    return target2[2]
            else:
                rospy.loginfo('pick failed')
                return False
    
    def look_around(self):
        bus_servo_control.set_servos(self.joints_pub, 200, ((1, 500), (2, 500)))
        rospy.sleep(0.2)
        if self.x_dis > 875 or self.x_dis < 125:
            self.d_pulse = -self.d_pulse
        bus_servo_control.set_servos(self.joints_pub, 50, ((6, self.x_dis),))           
        self.x_dis += self.d_pulse 
        rospy.sleep(0.05) 

    # Bring cube to center
    def go_home(self):
        bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))

    # moving to center on cube? or face?
    # moving to center on cube? or face?
    def center_target(self,img,center,area_max=900):
        (center_x,center_y) = center
        img_h, img_w = img.shape[:2]
        self.x_pid.SetPoint = img_w / 2.0  # 设定
        self.x_pid.update(center_x)  # 当前
        dx = self.x_pid.output
        self.x_dis += int(dx)  # 输出

        self.x_dis = 200 if self.x_dis < 200 else self.x_dis
        self.x_dis = 800 if self.x_dis > 800 else self.x_dis

        self.y_pid.SetPoint = 900  # 设定
        if abs(area_max - 900) < 50:
            area_max = 900
        self.y_pid.update(area_max)  # 当前
        dy = self.y_pid.output
        self.y_dis += dy  # 输出
        self.y_dis = 0.12 if self.y_dis < 0.12 else self.y_dis
        self.y_dis = 0.25 if self.y_dis > 0.25 else self.y_dis
        
        if abs(center_y - img_h/2.0) < 20:
            self.z_pid.SetPoint = center_y
        else:
            self.z_pid.SetPoint = img_h / 2.0
            
        self.z_pid.update(center_y)
        dy = self.z_pid.output
        self.z_dis += dy

        self.z_dis = 0.22 if self.z_dis > 0.22 else self.z_dis
        self.z_dis = 0.17 if self.z_dis < 0.17 else self.z_dis

        target = self.ik.setPitchRanges((0, round(self.y_dis, 4), round(self.z_dis, 4)), -90, -85, -95)
        if target:
            servo_data = target[1]
            # TODO: handle this joints pub thing
            bus_servo_control.set_servos(self.joints_pub, 20, (
                (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, self.x_dis)))
    
if __name__ == '__main__':
    motionControl = Movement()
