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

def orientation_cone():
    MOTORS = 1
    BODY = 0
    HEADTILT = 4

    tango = Controller()
    motors = 6000
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

    frames = pipeline.wait_for_frames()
    # Align the depth frame to color frame
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    headTilt = 4800
    tango.setTarget(HEADTILT, headTilt)
    counter = 0

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)

            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            orange_lower = np.array([0, 250, 50], np.uint8)
            orange_upper = np.array([60, 255, 255], np.uint8)
            #hsv = cv2.dilate(hsv,kernel)
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
            counter += 1
            if (counter > 50):
                
                #print((cv2.countNonZero(orange_mask) / orange_mask.size))
                if (((cv2.countNonZero(orange_mask) / orange_mask.size) < 0.005) or ((cv2.countNonZero(orange_mask) / orange_mask.size) > 0.5)):
                    motors = 5000
                    tango.setTarget(MOTORS,motors)
                    time.sleep(0.2)
                    motors = 6000
                    tango.setTarget(MOTORS,motors)

                else:
                    if (cX > 400):
                        motors -= 200
                        if(motors < 5200):
                            motors = 5200
                            tango.setTarget(MOTORS, motors)
                    elif (cX < 240):
                        motors += 200
                        if(motors > 6800):
                            motors = 6800
                            tango.setTarget(MOTORS, motors)
                    else:
                        if(distance > 1):
                            motors = 6000
                            tango.setTarget(MOTORS,motors)
                            body = 5000            
                            tango.setTarget(BODY,body)
                            time.sleep(0.3)
                            body = 6000            
                            tango.setTarget(BODY,body)
                        else:
                            body = 6000
                            tango.setTarget(BODY,body)
                            print ("In mining area")
                            return

    finally:

        # Stop streaming
        pipeline.stop()

def face_find():
    MOTORS = 1
    HEADTILT = 4
    HEADTURN = 3

    tango = Controller()
    motors = 6000
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

    face_cascade = cv2.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')

    headTurn = 6000
    tango.setTarget(HEADTURN, headTurn)
    headTilt = 5500
    tango.setTarget(HEADTILT, headTilt)


    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must 

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            cv2.waitKey(1)

            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            faces = face_cascade.detectMultiScale(gray, 1.2, 5,)

            if(len(faces) == 0):
                motors = 4800
                tango.setTarget(MOTORS,motors)
                time.sleep(0.2)
                motors = 6000
                tango.setTarget(MOTORS,motors)
            elif(len(faces) != 0):
                print("Found Face!")
                for (x,y,w,h) in faces:
                    cv2.rectangle(color_image,(x,y),(x+w,y+h),(255,0,0),2)
                return
                #cX = int((x + (w/2)))
                #cY = int((y + (h/2)))

                #distance = depth_frame.get_distance(cX,cY)

                #if (cX > 400):
                #    motors -= 200
                #    if(motors < 5000):
                #        motors = 5000
                #        tango.setTarget(MOTORS, motors)
                #elif (cX < 240):
                #    motors += 200
                #    if(motors > 7000):
                #        motors = 7000
                #        tango.setTarget(MOTORS, motors)
                #else:
                #    motors = 6000
                #    tango.setTarget(MOTORS, motors)

                #if(distance > 2):
                #    motors = 6000
                #    tango.setTarget(MOTORS,motors)
                #    body = 5200            
                #    tango.setTarget(BODY,body)
                #else:
                #    body = 6000
                #    tango.setTarget(BODY,body)
                #    print("Moved to Face!")
                #    return


    finally:

        # Stop streaming
        pipeline.stop()

def color_find():
    MOTORS = 1
    BODY = 0
    HEADTILT = 4
    HEADTURN = 3

    tango = Controller()
    motors = 6000
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
    color_image = np.asanyarray(color_frame.get_data())

    #yellow_lower = np.array([120, 150, 150], np.uint8)
    #yellow_upper = np.array([140, 255, 200], np.uint8)

    green_lower = np.array([40, 140, 40], np.uint8)
    green_upper = np.array([120, 255,120], np.uint8)

    pink_lower = np.array([125, 0, 125], np.uint8)
    pink_upper = np.array([255, 100, 255], np.uint8)

    savedColor = None


    try:
        while True:
             # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            cv2.waitKey(1)

            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            motors = 6000
            tango.setTarget(MOTORS,motors)
            body = 6000
            tango.setTarget(BODY,body)
            counter  = 0
            for x in range(10000):
                counter += 1

            #yellow_mask = cv2.inRange(color_image, yellow_lower, yellow_upper)
        
            green_mask = cv2.inRange(color_image, green_lower, green_upper)
        
            pink_mask = cv2.inRange(color_image, pink_lower, pink_upper)

            kernel = np.ones((5, 5), "uint8")
            
            #yellow_mask = cv2.dilate(yellow_mask, kernel)
            #res_yellow = cv2.bitwise_and(color_image, color_image, mask = yellow_mask)
            
            green_mask = cv2.dilate(green_mask, kernel)
            res_green = cv2.bitwise_and(color_image, color_image, mask = green_mask)
            
            pink_mask = cv2.dilate(pink_mask, kernel)
            res_pink = cv2.bitwise_and(color_image, color_image, mask = pink_mask)
        
            #contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
            #for pic, contour in enumerate(contours):
            #    area = cv2.contourArea(contour)
            #    if(area > 500):
            #        savedColor = "yellow"
            #        x, y, w, h = cv2.boundingRect(contour)
            #        color_image = cv2.rectangle(color_image, (x, y), (x + w, y + h), (51, 255, 255), 2)
                        

                    # Creating contour to track green color
            contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 500):
                    savedColor = "green"
                    x, y, w, h = cv2.boundingRect(contour)
                    color_image = cv2.rectangle(color_image, (x, y), 
                                            (x + w, y + h),
                                            (0, 255, 0), 2)
                        

            contours, hierarchy = cv2.findContours(pink_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
                    
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 500):
                    savedColor = "pink"
                    x, y, w, h = cv2.boundingRect(contour)
                    color_image = cv2.rectangle(color_image, (x, y),
                                            (x + w, y + h),
                                            (255, 77, 255), 2)

            savedColor = "pink"
            if(savedColor != None):
                print("COLOR DETECTED: " + savedColor)
                return savedColor
    finally:
        # Stop streaming
        pipeline.stop()

def goal_find(savedColor):
    MOTORS = 1
    BODY = 0
    HEADTILT = 4

    tango = Controller()
    motors = 6000
    body = 6000
    tango.setTarget(BODY, body)
    tango.setTarget(MOTORS, motors)

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

    goalColor = savedColor


    yellow_lower = np.array([120, 150, 150], np.uint8)
    yellow_upper = np.array([140, 255, 200], np.uint8)

    green_lower = np.array([40, 140, 40], np.uint8)
    green_upper = np.array([120, 255,120], np.uint8)

    pink_lower = np.array([125, 0, 125], np.uint8)
    pink_upper = np.array([255, 100, 255], np.uint8)

    count = 0
    target = 0
    headTilt = 5000
    tango.setTarget(HEADTILT, headTilt)

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)

            if goalColor == "pink":
                color_mask = cv2.inRange(color_image, pink_lower, pink_upper)
            elif goalColor == "yellow":
                color_mask = cv2.inRange(color_image, yellow_lower, yellow_upper)
            elif goalColor == "green":
                color_mask = cv2.inRange(color_image, green_lower, green_upper)

            Moments = cv2.moments(color_mask)
            if Moments["m00"] != 0:
                cX = int(Moments["m10"] / Moments["m00"])
                cY = int(Moments["m01"] / Moments["m00"])
            else:
                cX, cY = 0,0

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_mask)
            cv2.waitKey(1)

            cv2.circle(color_image, (cX, cY), 5, (0, 165, 255), -1)
            
            distance = depth_frame.get_distance(cX, cY)

            count += 1
            if (count > 50 and target == 0):
                #print((cv2.countNonZero(orange_mask) / orange_mask.size))
                if (((cv2.countNonZero(color_mask) / color_mask.size) < 0.01) or ((cv2.countNonZero(color_mask) / color_mask.size) > 0.5)):
                    motors = 5000
                    tango.setTarget(MOTORS,motors)
                    time.sleep(0.2)
                    motors = 6000
                    tango.setTarget(MOTORS,motors)

                else:
                    if (cX > 400):
                        motors -= 200
                        if(motors < 5200):
                            motors = 5200
                            tango.setTarget(MOTORS, motors)
                    elif (cX < 240):
                        motors += 200
                        if(motors > 6800):
                            motors = 6800
                            tango.setTarget(MOTORS, motors)
                    else:
                        motors = 6000
                        tango.setTarget(MOTORS, motors)
                        body = 6000
                        tango.setTarget(BODY, body)
                        target = 1
                        time.sleep(1)
            motors = 6000
            tango.setTarget(MOTORS, motors)
            body = 5200
            tango.setTarget(BODY, body)
            return

    finally:

        # Stop streaming
        pipeline.stop()

#orientation_cone()
#face_find()
result = color_find()
time.sleep(2)
goal_find(result)