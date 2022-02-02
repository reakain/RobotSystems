#! /usr/bin/python3
import rossros
from lane_follow_interpreter import LaneFollow
import time
from picarx_improved import Picarx
import Ultrasonic

def drive_car():

def camera_read(cam_bus, sensor_delay):
    camera = PiCamera()
    camera.resolution = (640,480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    for frame in camera.capture(rawCapture, format="bgr",use_video_port=True):# use_video_port=True
        cam_bus.set_message(frame)
        time.sleep(sensor_delay)

def camera_interpret(cam_bus,drive_bus, sensor_delay):
    lane_follow = LaneFollow()
    while(True):
        frame = cam_bus.get_message()
        steering_angle = lane_follow.get_heading(frame)
        drive_bus.set_message(steering_angle)
        time.sleep(sensor_delay)

def drive_car(input_busses, sensor_delay):
    px = Picarx()
    while(True):
        obstacle_distance = input_busses[1].get_message()
        if obstacle_distance < min_dist:
            speed = 0
        else:
            speed = 50
        steering_angle = input_busses[0].get_message()
        px.drive(speed,steering_angle)
        time.sleep(sensor_delay)


def get_obstacle_dist(ut_bus, sensor_delay):
    trig = Pin('D8')
    echo = Pin('D9')

    while(True):
        result = get_distance(trig, echo)
        if result != -1 and result != -2:
            ut_bus.set_message(result)
        time.sleep(sensor_delay)


def get_distance(trig,echo, timeout = 0.01):
    trig.low()
    time.sleep(0.01)
    trig.high()
    time.sleep(0.000015)
    trig.low()
    pulse_end = 0
    pulse_start = 0
    timeout_start = time.time()
    while echo.value()==0:
        pulse_start = time.time()
        if pulse_start - timeout_start > timeout:
            return -1
    while echo.value()==1:
        pulse_end = time.time()
        if pulse_end - timeout_start > timeout:
            return -2
    during = pulse_end - pulse_start
    cm = round(during * 340 / 2 * 100, 2)
    #print(cm)
    return cm


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
    find_lane = rossros.ConsumerProducer(camera_interpret, camera_bus, wheel_bus, name="Camera interpreter")
    drive_control = rossros.Consumer(drive_car, (wheel_bus, ut_bus), name = "Car Controller")

    rossros.runConcurrently()
