import pyrealsense2 as rs
import numpy as np
import cv2

def erosion(image):
    erosion_size = 21
    erosion_shape = cv2.MORPH_ELLIPSE
    
    element = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),(erosion_size, erosion_size))
    
    erosion_dst = cv2.erode(image, element)
    return erosion_dst

    
def dilatation(image):
    dilatation_size = 1
    dilation_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),(dilatation_size, dilatation_size))

    dilatation_dst = cv2.dilate(image, element)
    return dilatation_dst