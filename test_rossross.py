#! /usr/bin/python3
import rossros
from lane_follow_interpreter import LaneFollow
import time
from picarx_improved import Picarx

def drive_car():

def camera_read(cam_bus, sensor_delay):
    camera = PiCamera()
    camera.resolution = (640,480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    for frame in camera.capture(rawCapture, format="bgr",use_video_port=True):# use_video_port=True
        cam_bus.set_message(frame)
        time.sleep(sensor_delay)

def camera_interpret(input_busses,drive_bus, sensor_delay):
    lane_follow = LaneFollow()
    cam_bus = input_busses[0]
    drive_bus = input_busses[1]
    while(True):
        frame = cam_bus.get_message()
        current_drive = drive_bus.get_message()
        steering_angle = lane_follow.get_heading(frame)
        drive_bus.set_message((current_drive[0],steering_angle))
        time.sleep(sensor_delay)

def drive_car(drive_bus, sensor_delay):
    px = Picarx()
    while(True):
        current_drive = drive_bus.get_message()
        px.drive(current_drive[0],current_drive[1])
        time.sleep(sensor_delay)


def get_obstacle_dist(ut_bus, sensor_delay):
    pass

def get_grayscale(gray_bus, sensor_delay):
    pass


if __name__ == "__main__":
    camera_bus = rossros.Bus(name="PiCam Bus")
    gray_bus = rossros.Bus(name="Grayscale Bus")
    ut_bus = rossros.Bus(name="Ultra Sonic Bus")
    wheel_bus = rossros.Bus(name="Current wheel command")
    run_time = rossros.Timer(camera_bus,gray_bus,ut_bus, name="Data Harvester")

    get_gray = rossros.Producer(get_grayscale,gray_bus,name="Grayscale Reader")
    get_ut = rossros.Producer(get_obstacle_dist,ut_bus,name="UT Reader")
    get_cam = rossros.Producer(camera_read,camera_bus, name="Camera Reader")
    #parse_gray = rossros.ConsumerProducer(gray_bus)
    #parse_obst = rossros.ConsumerProducer(ut_bus)
    find_lane = rossros.ConsumerProducer(camera_interpret, (camera_bus, wheel_bus), wheel_bus, name="Camera interpreter")
    drive_control = rossros.Consumer(drive_car, wheel_bus, name = "Car Controller")

    rossros.runConcurrently()
