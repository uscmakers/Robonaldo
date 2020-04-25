#!/usr/bin/env python3
PKG = 'robonaldo_vision'
import roslib; roslib.load_manifest(PKG)

import time
import numpy as np
import cv2
import pyzed.sl as sl
import math
import queue

import rospy
from robonaldo_msgs.msg import ball_positions

float_max = np.finfo(np.float32).max

def ballDetect(image):
    #image = cv2.imread('soccer_pic/ball2.jpg')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Colors are in bgr
    lower = np.array([35, 110, 40], dtype = "uint8")
    upper = np.array([90, 255,200], dtype = "uint8")
    mask = cv2.inRange(hsv, lower, upper)

    imask = mask > 0
    green = np.zeros_like(image, np.uint8)
    green[imask] = image[imask]

    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntsSorted = sorted(contours, key = cv2.contourArea, reverse=True)
    cX = -1
    cY = -1

    ball_contour = None

    if len(cntsSorted) > 0 and cv2.contourArea(cntsSorted[0]) > 50:

        ball_contour = cntsSorted[0]

        cv2.drawContours(green, [ball_contour], 0, (0, 0, 255), 3)
        M = cv2.moments(ball_contour)
        cX = int(M['m10']/M['m00'])
        cY = int(M['m01']/M['m00'])

    """
    for contour in cntsSorted:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04*peri, True)
        if (len(approx) > 8 or len(approx) < 3) and cv2.contourArea(contour) > 200:
            cv2.drawContours(green, [contour], 0, (0, 0, 255), 3)
            M = cv2.moments(contour)
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            cv2.circle(green, (cX, cY), 7, (255, 0, 0), -1)
            print('center: ({},{})'.format(cX,cY))
            break
    """

    #print('center: ({},{})'.format(cX,cY))

    return green, cX, cY, ball_contour  

def findDepth(ball_contour, depth_map):
    dY, dX, depth = -1, -1, -1
    masked_depth = np.full(depth_map.shape, np.nan, dtype=np.float32)

    if ball_contour is not None:
        mask = np.zeros(depth_map.shape, np.uint8)
        cv2.drawContours(mask, [ball_contour], -1, (255), -1)
        imask = mask > 0.0

        masked_depth[imask] = depth_map[imask]
        masked_depth = np.nan_to_num(masked_depth, nan=float_max, posinf=float_max, neginf=float_max)

        dY, dX = np.unravel_index(masked_depth.argmin(axis=None), masked_depth.shape)
        depth = depth_map[dY, dX]

    return depth, dX, dY, masked_depth

    
if __name__ == '__main__':

    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use milliliter units (for depth measurements)
    init_params.depth_minimum_distance = 150

    init_params.camera_fps = 60
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.depth_minimum_distance = 300

    # Set sensing mode in FILL
    runtime_parameters =sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print('Error is ', err)
        exit(1)

    image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)
    image_depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)

    pub = rospy.Publisher('ball_position', ball_positions)
    rospy.init_node('vision_processing')
    rate = rospy.Rate(60) #loops 10 times per second

    timer_frames = 5

    timer_count = timer_frames
    start_time = time.time()

    # Video capturing
    while (not rospy.is_shutdown()) and zed.grab() == sl.ERROR_CODE.SUCCESS:

        # Retrieve left image in sl.Mat (which is the RGB camera values)
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)

        # Retrieve depth map
        zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
        # Load depth data into a numpy array
        depth_ocv = depth_zed.get_data()

        zed.retrieve_image(image_depth_zed, sl.VIEW.DEPTH)
        # Use get_data() to get the numpy array
        image_depth_ocv = image_depth_zed.get_data()

        # get_data() converts ZED object (Mat) to numpy array (for openCV)
        image_ocv = image_zed.get_data()
        green, cX, cY, ball_contour = ballDetect(image_ocv)
        depth, dX, dY, ball_depth = findDepth(ball_contour, depth_ocv)
        
        #print('depth_ocv.shape: ', depth_ocv.shape)
        print('depth at center: ', depth_ocv[cY][cX])
        #print('fixed depth at center: ', depth_ocv[dY][dX])        

        #point cloud
        point_cloud = sl.Mat()
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        err, point_cloud_value = point_cloud.get_value(cX, cY)

        print("X: {}, Y: {}, Z: {}".format(point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]))
        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1]  + point_cloud_value[2] * point_cloud_value[2])
        print('distance: ', distance)

        x = point_cloud_value[0]
        z = point_cloud_value[2]
        angle = math.atan(x/z)
        angle_degree = math.degrees(angle)
        print('Angle : {}'.format(angle_degree))
        
        #ball_depth[ball_depth==float_max] = 0
        #ball_depth = cv2.cvtColor(ball_depth, cv2.COLOR_GRAY2BGR)

        msg = ball_positions(angle=angle_degree, distance=distance)
        pub.publish(msg)
        rate.sleep()

        cv2.circle(green, (cX, cY), 5, (255, 0, 0), -1)
        cv2.circle(ball_depth, (dX, dY), 5, (0, 0, 255), -1)

        #note to self: pass image_ocv into ballDetect9()
        # Display left image
        cv2.imshow("Image", image_ocv)
        # cv2.imshow('depth', depth_ocv)
        # cv2.imshow('masked depth', ball_depth)
        cv2.imshow('green', green)
        cv2.imshow('image depth', image_depth_ocv)
        cv2.waitKey(1)

        timer_count -= 1
        if timer_count == 0:
            end_time = time.time()
            print("FPS:", int(timer_frames/(end_time - start_time)))
            timer_count = timer_frames
            start_time = end_time
