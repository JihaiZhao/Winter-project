# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import argparse
import os
from filter import erosion, dilatation
import time

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
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

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

#create trackbar
max_value_H = 180
max_value = 255
low_H = 100
low_S = 0
low_V = 0
high_H = 180
high_S = 255
high_V = 255
swindow_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)

cv2.namedWindow(window_detection_name, cv2.WINDOW_NORMAL)
cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        
        dpt = aligned_depth_frame.as_depth_frame() ###
        
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 120
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        ## only show purple
        hsv = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV) #background
        hsv = np.array(hsv)

        img2gray = cv2.cvtColor(bg_removed,cv2.COLOR_BGR2GRAY)

        lowerBound = np.uint8([[[low_H, low_S, low_V]]])
        upperBound = np.uint8([[[high_H, high_S, high_V]]])

        purple_mask = cv2.inRange(hsv, lowerBound, upperBound)
        masked_bg = cv2.bitwise_and(hsv, hsv, mask= purple_mask)

        # clean up masked image
        kernel_1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12))
        masked_image_opening = cv2.morphologyEx(masked_bg, cv2.MORPH_OPEN, kernel_1)
        kernel_2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(40,40))
        masked_image_clean = cv2.morphologyEx(masked_image_opening, cv2.MORPH_CLOSE, kernel_2)

        #contour detection
        color_image_gray = cv2.cvtColor(masked_image_clean, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(color_image_gray, 10, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #find the centroid of the object we observed  
        cx=0
        cy=0
        areas = []
        centroids = []
        locations = []
        if len(contours) > 0:
            for cnt in contours:
                M = cv2.moments(cnt)
                #print(M)
                area = M['m00']
                try:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])     
                    centroid = (cx,cy)
                    centroids.append(centroid)
                    areas.append(area)                
                    rect = cv2.minAreaRect(cnt)
                    print(rect)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    # cv2.drawContours(masked_image_clean,[box],0,(0,255,0),2) 
                    cv2.drawContours(color_image,[box],0,(0,255,0),2) 
                except:
                    pass  
                
            if areas:
                max = np.argmax(areas)
                centroid_max = centroids[max]
                pixel_distance_in_meters = dpt.get_distance(centroid_max[0], centroid_max[1]) #get the distace of the objhect from the cemra

                p = profile.get_stream(rs.stream.color)
                intr = p.as_video_stream_profile().get_intrinsics()
                location = rs.rs2_deproject_pixel_to_point(intr, centroid, pixel_distance_in_meters) 
                locations.append(location)
                # print(location)
                # cv2.circle(masked_image_clean, centroid_max, 5, [0,0,255], 5)    
                cv2.circle(color_image, centroid_max, 5, [0,0,255], 5)

        cv2.namedWindow(window_detection_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_detection_name, color_image)
        cv2.imshow('mask', purple_mask)
        # cv2.imshow(window_detection_name, masked_image_clean)      


        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop() 