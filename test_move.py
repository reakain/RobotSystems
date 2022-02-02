#! /usr/bin/python3
import sys
import signal
#sys.path.append(r'/home/pi/picar-x/lib')
#from utils import reset_mcu
#reset_mcu()

from picarx_improved import Picarx
from machine_vision import CarCam
import time
import logging


# Callback to handle SIGINT and SIGTERM
def shutdown_callback(_1, _2):
    # Make sure subprocesses are terminated
    # Log, then shut down remaining
    logging.log('System halted')
    ending = True
    sys.exit(0)


if __name__ == "__main__":
    # Allow keyboard exit
    signal.signal(signal.SIGINT, shutdown_callback)
    signal.signal(signal.SIGTERM, shutdown_callback)
    ending = False

    try:
        px = Picarx()
        picam = CarCam()
        
        ### Testing  Drives from part 2
        #logging.info("Testing Part 2 controls")
        #logging.info("Driving straight")
        #px.drive(50,0)
        #time.sleep(1)
        #px.stop()

        #logging.info("Starting parallel park")
        #px.parallel_park(100)

        #logging.info("Starting 3 point turn")
        #px.three_point_turn()

        ### Testing Part 3 Control
        logging.info("Testing Grayscale line follow")
        while not ending:
            px.turn_to_line_grayscale()
            
        logging.info("Testing camera line follow")
        while not ending:
            px.set_dir_servo_angle(picam.get_heading())


    finally:
        px.forward(0)


