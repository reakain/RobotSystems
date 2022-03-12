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
        
        #self.y_d = 0
        self.grasps=Grasp()
        self.roll_angle = 0
        self.gripper_rotation = 0
      # 木块对角长度一半
        self.square_diagonal = 0.03*math.sin(math.pi/4)
        #F = 1000/240.0
        #self.last_x_dis=x_dis
        config = rospy.get_param('config', {})
        if config != {}:    
            self.color_z_min = config['color_z_min']
            self.color_z_max = config['color_z_max']
        self.Y = 0
        self.X = 0

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
        self.Y = 0
        self.X = 0

    # Do the grabber clacky clacky, can probably use this to drop the cube too lolol
    def clacky_clacky(self):
        bus_servo_control.set_servos(self.joints_pub, 300, ((1, 50),(2, 300),))
        rospy.sleep(0.3)

        bus_servo_control.set_servos(self.joints_pub, 600, ((1, 500),(2, 700),))
        rospy.sleep(0.6)
        
        bus_servo_control.set_servos(self.joints_pub, 600, ((1, 50),(2, 300),))
        rospy.sleep(0.6)
        
        bus_servo_control.set_servos(self.joints_pub, 300, ((1, 500),(2, 500),))
        rospy.sleep(0.3)

        bus_servo_control.set_servos(self.joints_pub, 600, ((1,50),))
        rospy.sleep(0.6)

        bus_servo_control.set_servos(self.joints_pub, 600, ((1,500),))
        rospy.sleep(0.6)

        bus_servo_control.set_servos(self.joints_pub, 600, ((1,50),))
        rospy.sleep(0.6)

        bus_servo_control.set_servos(self.joints_pub, 600, ((1,500),))
        #rospy.sleep(0.4)
        
        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 100),))
        # rospy.sleep(0.4)

        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 500),))
        # rospy.sleep(0.4)
        
        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 100),))
        # rospy.sleep(0.4)
        
        # bus_servo_control.set_servos(self.joints_pub, 400, ((1, 500),))
        rospy.sleep(1)

    
    def grasp_hell(self, centers, max_area, box_rotation_angle, d_color_map=30, color_y_adjust=400, d_color_y=20):
        centerX,centerY = centers
  
        if 298 + d_color_map < centerY <= 424 + d_color_map:
            self.Y = Misc.map(centerY, 298 + d_color_map, 424 + d_color_map, 0.12, 0.12 - 0.04)
        elif 198 + d_color_map < centerY <= 298 + d_color_map:
            self.Y = Misc.map(centerY, 198 + d_color_map, 298 + d_color_map, 0.12 + 0.04, 0.12)
        elif 114 + d_color_map < centerY <= 198 + d_color_map:
            self.Y = Misc.map(centerY, 114 + d_color_map, 198 + d_color_map, 0.12 + 0.08, 0.12 + 0.04)
        elif 50 + d_color_map < centerY <= 114 + d_color_map:
            self.Y = Misc.map(centerY, 50 + d_color_map, 114 + d_color_map, 0.12 + 0.12, 0.12 + 0.08)
        elif 0 + d_color_map < centerY <= 50 + d_color_map:
            self.Y = Misc.map(centerY, 0 + d_color_map, 50 + d_color_map, 0.12 + 0.16, 0.12 + 0.12)
        else:
            self.Y = 1

        self.x_pid.SetPoint = 340 #设定           
        self.x_pid.update(centerX) #当前
        dx = self.x_pid.output
        self.x_dis += dx #输出  
        
        self.x_dis = 0 if self.x_dis < 0 else self.x_dis          
        self.x_dis = 1000 if self.x_dis > 1000 else self.x_dis

        self.y_pid.SetPoint = color_y_adjust
        centerY += abs(Misc.map(70*math.sin(math.pi/4)/2, 0, self.size[0], 0, img_w)*math.sin(math.radians(abs(self.gripper_rotation) + 45))) + 65*math.sin(math.radians(abs(self.roll_angle)))
        if self.Y < 0.12 + 0.04:
            centerY += d_color_y 
        if 0 < centerY - color_y_adjust <= 5:
            centerY = color_y_adjust
        self.y_pid.update(centerY)

        dy = self.y_pid.output
        self.y_dis += dy
        self.y_dis = 0.1 if self.y_dis > 0.1 else self.y_dis
        self.y_dis = -0.1 if self.y_dis < -0.1 else self.y_dis
        rotation_angle = 240 * (self.x_dis - 500)/1000.0
        self.X = round(-self.Y * math.tan(math.radians(rotation_angle)), 4)

        

    def approach_cube(self):
        # 夹取的位置
        self.grasps.grasp_pos.position.x = self.X
        self.grasps.grasp_pos.position.y = self.Y
        self.grasps.grasp_pos.position.z = Misc.map(self.Y - 0.15, 0, 0.15, self.color_z_min, self.color_z_max)
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
    def grab_cube(self, box_rotation_angle, have_adjust=False):

        position = self.grasps.grasp_pos.position
        rotation = self.grasps.grasp_pos.rotation
        approach = self.grasps.grasp_approach
        retreat = self.grasps.grasp_retreat
        #y_d = 0
        #roll_angle = 0
        square_diagonal = 0.03*math.sin(math.pi/4)
        F = 1000/240.0
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


                self.roll_angle = target2[2]
                self.gripper_rotation = box_rotation_angle #this uses the max contour stuff 

                self.x_dis = self.last_x_dis = target2[1]['servo6']
                self.y_dis  = 0


            else:
                # 第五步: 对齐
                bus_servo_control.set_servos(self.joints_pub, 500, ((2, 500 + int(F * self.gripper_rotation)),))
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
        #bus_servo_control.set_servos(self.joints_pub, 200, ((1, 500), (2, 500)))
        #rospy.sleep(0.2)
        if self.x_dis > 875 or self.x_dis < 125:
            self.d_pulse = -self.d_pulse
        bus_servo_control.set_servos(self.joints_pub, 100, ((6, self.x_dis),))           
        self.x_dis += self.d_pulse 
        rospy.sleep(0.05) 

    # Bring cube to center
    def go_home(self):
        #servo_data = self.ik.setPitchRanges((0, 0.17, 0.3), -65, -180, 0)[1]
        bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 75), (2, 500), (3, 300), (4, 825), (5, 625), (6, 500)))
        #bus_servo_control.set_servos(self.joints_pub, 1500, (
        #    (1, 400), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']),
        #    (6, servo_data['servo6'])))

    # moving to center on cube? or face?
    # moving to center on cube? or face?
    def center_target(self,img_shape,center,area_max=900):
        (center_x,center_y) = center
        img_h, img_w = img_shape
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
            bus_servo_control.set_servos(self.joints_pub, 20, (
                (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, self.x_dis)))
    
if __name__ == '__main__':
    motionControl = Movement()
