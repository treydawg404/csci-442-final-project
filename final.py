## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

#Nathan Parnell, Trey Grossman

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import math
from maestro import Controller                                                     

def oreitnation_cone():
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

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    frames = pipeline.wait_for_frames()
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            orange_lower = np.array([0, 250, 20], np.uint8)
            orange_upper = np.array([60, 255, 255], np.uint8)
            orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)
            Moments = cv2.moments(orange_mask)
            if Moments["m00"] != 0:
                cX = int(Moments["m10"] / Moments["m00"])
                cY = int(Moments["m01"] / Moments["m00"])
            else:
                cX, cY = 0,0

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', orange_mask)
            cv2.waitKey(1)

            cv2.circle(color_image, (cX, cY), 5, (0, 165, 255), -1)
            
            distance = depth_frame.get_distance(cX, cY)

            if (cX == 0 and cY == 0):
                motors = 5200
                body = 0
                tango.setTarget(MOTORS, motors)
                tango.setTarget(BODY, body)
            else:
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

                if(distance > 1):
                    motors = 6000
                    tango.setTarget(MOTORS,motors)
                    body = 5200            
                    tango.setTarget(BODY,body)
                else:
                    body = 6000
                    tango.setTarget(BODY,body)

            print(distance)


    finally:

        # Stop streaming
        pipeline.stop()

oreitnation_cone()