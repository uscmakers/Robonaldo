#!/usr/bin/env python
PKG = 'robonaldo_vision'
import roslib; roslib.load_manifest(PKG)

import numpy as np
import cv2
import pyzed.sl as sl
import math

import rospy
from robonaldo_msgs.msg import ball_positions


def ballDetect(image, depth):
    #image = cv2.imread('soccer_pic/ball2.jpg')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([36, 25, 25], dtype = "uint8")
    upper = np.array([70, 255, 255], dtype = "uint8")
    mask = cv2.inRange(hsv, lower, upper)

    imask = mask > 0
    #print(imask)
    green = np.zeros_like(image, np.uint8)
    green[imask] = image[imask]

    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntsSorted = sorted(contours, key = cv2.contourArea, reverse=True)
    cX = -1
    cY = -1
    for contour in cntsSorted:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04*peri, True)
        if len(approx) == 4 and cv2.contourArea(contour) > 200:
            cv2.drawContours(green, [contour], 0, (0, 0, 255), 3)
            M = cv2.moments(contour)
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            cv2.circle(green, (cX, cY), 7, (0, 0, 255), -1)
            print('center: ({},{})'.format(cX,cY))
            break
            
        

    
#    temp = np.full(depth.shape, np.inf)
#    temp[~imask] = depth[~imask]

#   print("min distance: " , temp.flatten()[np.argmin(temp.flatten())])
    return green, cX, cY

#    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    cnts = imutils.grab_contours(cnts)
#    center = None
#    
#    if len(cnts > 0:
#        c = max(cnts, key = cv2.contourArea)
#        ((x, y), radius) = cv2.minEnclosingCircle(c)
#        M = cv2.moments(c)
#        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

#        if radius > 10:
#            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
#            cv2.circle(frame, center, 5, (0, 0, 255), -1)


    #cv2.imwrite('curr.png', green)


if __name__ == '__main__':

    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use milliliter units (for depth measurements)

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print('Error is ', err)
        exit(1)

    image_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.U8_C4)
    depth_zed = sl.Mat(zed.get_camera_information().camera_resolution.width, zed.get_camera_information().camera_resolution.height, sl.MAT_TYPE.F32_C1)

    pub = rospy.Publisher('ball_position', ball_positions)
    rospy.init_node('vision_processing')
    rate = rospy.Rate(10) #loops 10 times per second

    # Video capturing
    while (not rospy.is_shutdown()) and zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image in sl.Mat (which is the RGB camera values)
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)

        # Retrieve depth map
        zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
        # Load depth data into a numpy array
        depth_ocv = depth_zed.get_data()

        # get_data() converts ZED object (Mat) to numpy array (for openCV)
        image_ocv = image_zed.get_data()
        green, cX, cY = ballDetect(image_ocv, depth_ocv) 
        
        print('depth_ocv.shape: ', depth_ocv.shape)
        print('depth at center: ', depth_ocv[cY][cX])        

        msg = ball_positions(angle=cX, distance=cY)
        pub.publish(msg)
        rate.sleep()

        #point cloud
#        point_cloud = sl.Mat()
#        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

#        err, point_cloud_value = point_cloud.get_value(cX, cY)

#        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1]  + point_cloud_value[2] * point_cloud_value[2])
#        print('distance: ', distance)
        
        #note to self: pass image_ocv into ballDetect9()
        # Display left image
        cv2.imshow("Image", image_ocv)
        cv2.imshow('depth', depth_ocv)
        cv2.imshow('green', green)
        cv2.waitKey(1)




    
    
