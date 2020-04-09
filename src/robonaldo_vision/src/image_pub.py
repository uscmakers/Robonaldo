#!/usr/bin/env python
PKG = 'image_processing'
import roslib; roslib.load_manifest(PKG)
import cv2
import sys

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy
def talker():
    pub = rospy.Publisher('image', numpy_msg(numpy.uint8),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz

    cap = cv2.VideoCapture(3) 
    while(1):
        ret, frame = cap.read()
        #print(height)
        #cv2.imshow("Cropped Image", crop_img)
        #gray =+ cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('frame', frame)
        #cv2.waitKey(10)
	#print("shape:", frame.shape)   
	#print("frametype:", frame.dtype)      
	a = numpy.array(frame, dtype=numpy.uint8).flatten()		
        #print(a)
        #a = a.reshape(480, 640, 3)
	#print("reshape:", a.shape)        
        #cv2.imshow('frame', a)
        cv2.waitKey(10)

        #print(numpy.array_equal(frame, a))
        pub.publish(a)

    try:
        while(0):
            ret, frame = cap.read()
            #print(height)
            #cv2.imshow("Cropped Image", crop_img)
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #print(type(frame))
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print 'Interrupted'
        sys.exit(0)



    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    talker()
