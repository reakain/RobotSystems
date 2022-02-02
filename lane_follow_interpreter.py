#! /usr/bin/python3
# coding=utf-8
# Stolen from https://github.com/sunfounder/picar-x/blob/v2.0/example/color_detect.py
# And shamelessly stolen from https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import logging
import math

lower_blue = np.array([60,40,40])
upper_blue = np.array([150,255,255])

class LaneFollow(object):
    def __init__(self, lower_color=lower_blue, upper_color=upper_blue):
        #init camera
        logger.info("start cam control")
        self.camera.resolution = (640,480)
        self.lower_color = lower_color
        self.upper_color = upper_color

    def get_heading(self,frame):
        img = frame.array
        lane_lines = self.detect_lane(img)
        if len(lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
            return -99

        height, width, _ = img.shape

        if len(lane_lines) == 1:
            logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid

        # find the steering angle, which is angle between navigation direction to end of center line
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

        logging.debug('new steering angle: %s' % steering_angle)
        return steering_angle

    def detect_lane(self,img):
        edges = self.find_edges(img)
        cropped_edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.average_slope_intercept(img, line_segments)

        return lane_lines
    
    def find_edges(self,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        edges = cv2.Canny(mask, 200, 400)

        return edges
    
    def region_of_interest(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only look at the bottom half (closer to wheels)
        polygon = np.array([[
            (0, height*1/2),
            (width, height*1/2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)

        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges

    def detect_line_segments(self,cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                        np.array([]), minLineLength=8, maxLineGap=4)

        return line_segments

    
    def average_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            logging.info('No line_segment segments detected')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(make_points(frame, right_fit_average))

        logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

        return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]