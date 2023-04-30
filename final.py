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

time.sleep(1)

tracker = cv2.TrackerKCF_create()

frames = pipeline.wait_for_frames()
# Align the depth frame to color frame
aligned_frames = align.process(frames)
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

# Convert images to numpy arrays
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())

bbox = cv2.selectROI(color_image, False)
tracker.init(color_image,bbox)


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

        ok, bbox = tracker.update(color_image)

        # Tracking success
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(color_image, p1, p2, (255,0,0), 2, 1)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        blank_image2 = 0 * np.ones(shape=[480, 1280, 3], dtype=np.uint8)
        cv2.circle(blank_image2, (640,math.floor(240 - (depth_frame.get_distance( math.floor((bbox[0]+bbox[2])/2), math.floor((bbox[1]+bbox[3])/2) )*100))), (5), (255, 255, 0), 2, 1)
        cv2.circle(blank_image2, (640,240), (5), (0, 0, 255), 2, 1)
        images = np.vstack((images,blank_image2))


        print (depth_frame.get_distance( math.floor((bbox[0]+bbox[2])/2), math.floor((bbox[1]+bbox[3])/2) )*100)

        xCoord = (bbox[0] + bbox[2]/2)
        xCoord = math.floor(xCoord)
        yCoord = (bbox[1]+ bbox[3]/2)
        yCoord = math.floor(yCoord)
        distance = depth_frame.get_distance(xCoord,yCoord)
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        if(distance < 1):
            body += 200
            if(body >7900):
                   body = 7900
            tango.setTarget(BODY,body)
        elif(distance > 1):
            body -= 200
            if(body < 1510):
                body = 1510
            tango.setTarget(BODY,body)

            



finally:

    # Stop streaming
    pipeline.stop()
