## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

#Nathan Parnell, Trey Grossman

def orient_finish():
    while(1):
         
        print("start orient to mining area")
        orange_lower = np.array([0, 50, 20], np.uint8)
        orange_lower = np.array([0, 200, 20], np.uint8)
        orange_upper = np.array([60, 255, 255], np.uint8)
        orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
        Moments = cv2.moments(orange_mask)
        if Moments["m00"] != 0:
                    cX = int(Moments["m10"] / Moments["m00"])
                    cY = int(Moments["m01"] / Moments["m00"])
        else:
                    cX, cY = 0,0
        cv2.circle(color_image, (cX, cY), 5, (0, 165, 255), -1)

        distance = depth_frame.get_distance(cX,cY)


        if (cX > 370):
            motors -= 200
            if(motors < 5000):
                motors = 5000
                tango.setTarget(MOTORS, motors)
        elif (cX < 270):
            motors += 200
            if(motors > 7000):
                motors = 7000
                tango.setTarget(MOTORS, motors)
        else:
            motors = 6000
            tango.setTarget(MOTORS, motors)

        if(distance > 1.5):
            body = 5200
            tango.setTarget(MOTORS,motors)
            tango.setTarget(BODY,body)
        else:
            body = 6000
            tango.setTarget(BODY,body)
            print("Entered Mining Area!")
            return



import pyrealsense2 as rs
import numpy as np
import cv2
import time
from maestro import Controller

MOTORS = 1
TURN = 2
BODY = 0

tango = Controller()
motors = 6000
turns = 6000
body = 6000

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

frames = pipeline.wait_for_frames()
# Align the depth frame to color frame
aligned_frames = align.process(frames)
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

# Convert images to numpy arrays
color_image = np.asanyarray(color_frame.get_data())



orient_finish()

# Stop streaming
pipeline.stop()
