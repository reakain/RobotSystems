import sys
#from sim.utils import reset_mcu
#reset_mcu()

from picarx_improved import Picarx
import time

#def parallel_parking(side = "left"):

def three_point_turn(initial = "left"):
    current_dir = 1
    speed = 100
    angle = 90
    if initial == "right":
        current_dir = -1
    px.drive(speed, current_dir*angle)
    time.sleep(0.5)
    px.stop()
    current_dir = -1 * current_dir
    px.drive(-speed, current_dir*angle)
    time.sleep(0.5)
    px.stop()
    current_dir = -1 * current_dir
    px.drive(speed, current_dir*angle)
    time.sleep(0.5)
    px.drive(speed,0)
    px.stop()




if __name__ == "__main__":
    try:
        px = Picarx()
        px.forward(30)
        time.sleep(0.5)
        for angle in range(0,35):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)        
        for angle in range(-35,0):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        px.forward(0)
        time.sleep(1)

        for angle in range(0,35):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)        
        for angle in range(-35,0):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(0,35):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)        
        for angle in range(-35,0):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
            
    finally:
        px.forward(0)


