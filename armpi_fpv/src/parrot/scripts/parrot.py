#!/usr/bin/python3
# coding=utf8
# This is the code that would actually run a ros node maybe? idk bro

import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import RLock, Timer

from std_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from warehouse.msg import Grasp
from parrot.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

# Our custom classes for analysis and movement
import Movement
import Perception

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

perception = Perception.Perception()

lock = RLock()

__findingFace = True
__isRunning = False
__goHome = False
__dropCube = False

stop_state = 0
move_state = 1

heartbeat_timer = None
org_image_sub_ed = False

def initMove(delay=True):
    with lock:
        movement.go_home()
    if delay:
        rospy.sleep(2) 

# app初始化调用
def init():
    global stop_state
    global __target_data

    rospy.loginfo("parrot Init")
    stop_state = 0
    __target_data = ((), ())
    initMove()
    #reset()

def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("enter parrot")
    with lock:
        init()
        if not org_image_sub_ed:
            org_image_sub_ed = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
            
    return [True, 'enter']

def exit_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed
    
    rospy.loginfo("exit parrot")
    with lock:
        __isRunning = False
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass
    
    return [True, 'exit']

def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running parrot")
    with lock:
        __isRunning = True

def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running parrot")
    with lock:
        __isRunning = False
        reset()
        initMove(delay=False)

def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
    
    return [True, 'set_running']


__target_data = ((), ())
def set_target(msg):
    global lock
    global __target_data
    
    rospy.loginfo('%s', msg)
    with lock:
        __target_data = (msg.color, msg.tag)

    return [True, 'set_target']

def heartbeat_srv_cb(msg):
    global heartbeat_timer
    
    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/parrot/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

def run(img):
    global __findingFace
    global __findingCube
    global __goHome
    global __dropCube

    if __findingFace:
        img, center = perception.FindFace(img)

        if center != None:
            movement.center_target(img,center)
            movement.clacky_clacky()
            __findingFace = False
            __findingCube = True
    elif __findingCube:
        if len(__target_data[0]) != 0:
            img, center = perception.FindColorCube(img, __target_data[0])

            if center != None:
                movement.center_target(img,center)
                movement.grab_cube()
                __findingCube = False
                __goHome = True
    elif __goHome:
        movement.go_home()
        __goHome = False
        __dropCube = True
    elif __dropCube:
        movement.clacky_clacky()
        __dropCube = False
        __findingFace = True

    return img


def image_callback(ros_image):
    global lock
    global stop_state

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # 转为opencv格式
    frame = cv2_img.copy()
    frame_result = frame
    
    with lock:
        if  __isRunning:
            frame_result = run(frame)
        else:
            if stop_state:
                stop_state = 0
                initMove(delay=False)
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring() # 转为ros格式
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


if __name__ == '__main__':
    # Init stuff
    rospy.init_node('parrot', log_level=rospy.DEBUG)

    # Get our publish and subscribe bits
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    image_pub = rospy.Publisher('/parrot/image_result', Image, queue_size=1)  # register result image publisher

    # We'll maybe use these service ones??
    enter_srv = rospy.Service('/parrot/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/parrot/exit', Trigger, exit_func)
    running_srv = rospy.Service('/parrot/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/parrot/heartbeat', SetBool, heartbeat_srv_cb)

    movement = Movement.Movement(joints_pub)
    perception = Perception.Perception()
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})

    #debug = False
    #if debug:
    #    enter_func(1)
    #    start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()