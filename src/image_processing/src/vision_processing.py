import numpy as np
import cv2
import pyzed.sl as sl

def ballDetect(image, depth):
    #image = cv2.imread('soccer_pic/ball2.jpg')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([36, 25, 25], dtype = "uint8")
    upper = np.array([70, 255, 255], dtype = "uint8")
    mask = cv2.inRange(hsv, lower, upper)
    print(depth)
    imask = mask > 0
    #print(imask)
    green = np.zeros_like(image, np.uint8)
    green[imask] = image[imask]
    
#    temp = np.full(depth.shape, np.inf)
#    temp[~imask] = depth[~imask]

#    print("min distance: " , temp.flatten()[np.argmin(temp.flatten())])
    return green

    


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


    # Video capturing
    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image in sl.Mat (which is the RGB camera values)
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)

        # Retrieve depth map
        zed.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
        # Load depth data into a numpy array
        depth_ocv = depth_zed.get_data()

        # get_data() converts ZED object (Mat) to numpy array (for openCV)
        image_ocv = image_zed.get_data()
        green = ballDetect(image_ocv, depth_ocv)        
        #note to self: pass image_ocv into ballDetect9()
        # Display left image
        cv2.imshow("Image", image_ocv)
        cv2.imshow('depth', depth_ocv)
        cv2.imshow('green', green)
        cv2.waitKey(1)



    
    
