import sys
#sys.path.append(r'/home/pi/picar-x/lib')
#from utils import reset_mcu
#reset_mcu()

from picarx_improved import Picarx
import time
import logging

def parallel_parking(car, side = "left"):
    current_dir = 1
    speed = 100
    angle = 45
    #if initial == "right":
    #    current_dir = -1
    px.drive(-speed, angle)
    time.sleep(0.5)
    px.stop()
    px.drive(-speed, -angle)
    time.sleep(0.5)
    px.stop()
    px.drive(speed, 20)
    time.sleep(0.5)
    px.stop()
    px.drive(speed, 0)
    time.sleep(0.5)
    px.stop()

def three_point_turn(initial = "left"):
    current_dir = -1
    speed = 100
    angle = 45
    if initial == "right":
        current_dir = 1
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
        logging.info("Driving straight")
        px.drive(50,0)
        time.sleep(1)
        px.stop()

        logging.info("Starting parallel park")
        px.parallel_park()

        logging.info("Starting 3 point turn")
        px.three_point_turn()


        # px.forward(30)
        # time.sleep(0.5)
        # for angle in range(0,35):
        #     px.set_dir_servo_angle(angle)
        #     time.sleep(0.01)
        # for angle in range(35,-35,-1):
        #     px.set_dir_servo_angle(angle)
        #     time.sleep(0.01)        
        # for angle in range(-35,0):
        #     px.set_dir_servo_angle(angle)
        #     time.sleep(0.01)
        # px.forward(0)
        # time.sleep(1)

        # for angle in range(0,35):
        #     px.set_camera_servo1_angle(angle)
        #     time.sleep(0.01)
        # for angle in range(35,-35,-1):
        #     px.set_camera_servo1_angle(angle)
        #     time.sleep(0.01)        
        # for angle in range(-35,0):
        #     px.set_camera_servo1_angle(angle)
        #     time.sleep(0.01)
        # for angle in range(0,35):
        #     px.set_camera_servo2_angle(angle)
        #     time.sleep(0.01)
        # for angle in range(35,-35,-1):
        #     px.set_camera_servo2_angle(angle)
        #     time.sleep(0.01)        
        # for angle in range(-35,0):
        #     px.set_camera_servo2_angle(angle)
        #     time.sleep(0.01)
            
    finally:
        px.forward(0)


