#!/usr/bin/env python
PKG = 'image_processing'
import roslib; roslib.load_manifest(PKG)
import cv2
import numpy as np

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

image = np.zeros((480, 640, 3),dtype=np.uint8)

def callback(data):
    print rospy.get_name(), "I heard %s"%str(data.data)
    print("shape: ", data.data)
    
    global image
    image = data.data.reshape(480, 640, 3)
    print("dtype:", image.dtype)
    #cv2.imshow('image', image)


def listener():
    rospy.init_node('listener')
    rospy.Subscriber("image", numpy_msg(np.uint8), callback)    
    while True:
		
        cv2.imshow('image', image)
    	cv2.waitKey(10)
    

if __name__ == '__main__':
    listener()
