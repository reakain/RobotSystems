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

from std_srvs.srv import *
from sensor_msgs.msg import Image
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

# Our custom classes for analysis and movement
import Movement
import Perception

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

perception = Perception.Perception()

lock = RLock()

# app初始化调用
def init():
    rospy.loginfo("parrot Init")
    movement.go_home()
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


#def run():


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