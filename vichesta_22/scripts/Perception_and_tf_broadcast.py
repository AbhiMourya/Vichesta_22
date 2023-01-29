#!/usr/bin/env python3


import cv2                                          #for importing opencv 
import numpy as np                                  #useful in the array operations
import roslib                                       #roslib is an internal library to write tools
import sys                          #module to handle runtime environment
import rospy                            #for use of pytho with ROS
from sensor_msgs.msg import Image               #importing the image type sensor message
from cv_bridge import CvBridge, CvBridgeError           #for importing CvBridge used to convert the ROS format images to opencv format
import tf2_ros                  #imporing tf2
import geometry_msgs.msg            #for geometry messages
import tf_conversions



######## COLOR THRESHOLDING AND FILTERING ########
'''
h_min ----> Min value of Hue
s_min ----> Min value of Saturation
v_min ----> Min value of Value
h_max ----> Max value of Hue
s_max ----> Max value of Saturation
v_max ----> Max value of Value
'''

tomatoColor = [[0, 230, 80, 20, 255, 255]]       #Defining the HSV color space of red tomatoes with format [h_min, s_min, v_min, h_max, s_max, v_max]

###################################################


### GLOBAL VARIABLE ####

imgContour = None                           #imgContour is declared of the type None and it will hold the final image
depth_image = np.empty((480,640),float)     #this is a 2D array that hold the depth value of every pixel of the frame
midX = 0                                #this is the middle value of the entire span of the contour in the x - axis
midY = 0                        #this is the middle value of the entire span of the contour in the y - axis

#######################



def callback_depth(data_depth):             #this callback is for accessing the depth image

    global depth_image                  #this is to update the value in the global depth_image variable

    try:
        bridgeDepth = CvBridge()                #for creating the bridge object
        depth_image = bridgeDepth.imgmsg_to_cv2(data_depth, "32FC1") #for converting the depth image into depth matrix if 32FC1 encoding  ----> each pixel value is stored as one channel floating point with single precision
    except CvBridgeError as e:          #for handling errors
        print(e)





def getContours(img):           #this function is used to draw contours in their appropriate places
    global imgContour  
    global midX                 
    global midY
    global depth_image
    depth = 0                   #initial value of depth is taken to be 0
    cx = 320.5                  #the mid point of the x-axis is 320.5
    cy = 240.5                  #the mid point of the y-axis is 240.5
    fx = 554.387                #the focal length for x is 554.387
    fy = 554.387                #the focal length for y is 554.387
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)       #this is for getting all the contours that can be formed on the image
    x, y, w, h = 0, 0, 0, 0     #x, y, w, h are for getting the bounding rectangle dimensions for the drawing the contour
    midX = 0            #mid of x is set to 0
    midY = 0            #mid of y is set to 0
    counter = 0  #this counter is used to label the child frame ------> that means it's value is used to give unique label to every contour and hence giving unique value to every tomato
    for cnt in contours:                #for loop is for iterating over all the contours that needs to be drawn
        area = cv2.contourArea(cnt)         #this function is used to return the area of every contour that can be drawn on the image ---> that means every red tomato visible currently
        if area > 10:                   #this is for filtering the noise ---> that means if the area is >10 then it means we are looking at something substantial and we need to draw contour on that and broadcast the TF for that
            peri = cv2.arcLength(cnt, True)     #this is to get the perimeter of the contour

            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)       #0.02 is a factor ---> it can be adjusted for better detection

            x, y, w, h = cv2.boundingRect(approx)               #this returns and stores the value of x, y, w, h

            cv2.circle(imgContour, (x+(w // 2), y+(h // 2)), max(h // 2, w // 2), (255, 0, 0), 2)   #this is the circle drawing function that draws image on imgContour with center x+(w/2) and y+(h/2) the next parameter is to get the radius of the circle that is the max of the entire spread
            cv2.circle(imgContour, (x+(w // 2), y+(h // 2)), 1, (255, 0, 0), -1)    #the (255, 0, 0) ----> is in the BGR format so max of blue and 0 values of red and green makes the circle blue in colour
            midX = x + w // 2       #this is the mid of the circle
            midY = y + h // 2       #this is the mid of the circle


            if midX != 0 and midY != 0 and midX <= 640 and midY <= 480:     #as the frame of the output image is 640 x 480 pixels so this is to prevent accessing values that are out of bound by any case
                depth = depth_image[midY][midX]         #the depth value is registered for the point (midX, midY)

            X = depth*((midX-cx)/fx)            #conversion to the world coordinates
            Y = depth*((midY-cy)/fy)            #conversion to the world coordinates
            Z = depth                           #conversion to the world coordinates


            br = tf2_ros.TransformBroadcaster()     #setting up the TF2 broadcaster
            t = geometry_msgs.msg.TransformStamped()        #broadcasting is stamped for every object
            t.header.stamp = rospy.Time.now()       #the head stamp is the current time that we use this makes it unique
            t.header.frame_id = "camera_link2"          #as the camera on the arm has the camera_link2 so we are using that
            t.child_frame_id = "obj"+str(counter)           #this is the naming convention where the is given as obj + value of the counter -----> obj1, obj2 etc.

            cv2.putText(imgContour, t.child_frame_id,               #this function is used to put text on the imgContour, the text is the child_frame_id, at the point (midX, midY) 
            (midX, midY), cv2.FONT_HERSHEY_SIMPLEX,         #cv2.FONT_HERSHEY_SIMPLEX ----> is the font used to label the image
            0.5, (255, 0, ), 2)             #0.5 is the font scale, (255, 0, 0) is for giving blue colour and 2 is the thickness


            t.transform.translation.x = Z           #this is for transforming the world coordinates to the camera frame that is on the arm
            t.transform.translation.y = -X          #this is for transforming the world coordinates to the camera frame that is on the arm
            t.transform.translation.z = Y           #this is for transforming the world coordinates to the camera frame that is on the arm

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)       #for conversion euler to quaternion
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)         #for broadcating the TF 

            counter = counter+1 #as this loop loops through the number of times equal to the number of unique contours that can be drawn then if the counter is incremented same number of times it will have unique value starting from 1 for every contour

    cv2.imshow("Result",imgContour)         #this is for displaying the final image with all the contours on it
    cv2.waitKey(1)          #this is for adding a 1 millisecond delay

    return x + w // 2, y + h // 2  # for mid of the box is returned using this



def callback(data):         #this callback is for color detection
    global imgContour     
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")      #the ROS format image is converted to bgr8 format -----> that is the format that is used in opencv
        imgContour = frame.copy()               #this function is used to copy the frame image to the imgContour
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         #this is used to convert the image to HSV image
        x, y = 0, 0
        for color in tomatoColor:       #this for loop goes through the tomatoColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
            lower = np.array(color[0:3])        #this is for forming the lower range of the HSV colour space ----> i.e. h_min, s_min, v_min
            upper = np.array(color[3:6])        #this is for forming the upper range of the HSV colour space ----> i.e. h_max, s_max, v_max
            mask = cv2.inRange(imgHSV, lower, upper)            #for masking the image 
            x, y = getContours(mask)   
    except CvBridgeError as e:          #for handling the error
        print(e)



def main(args):                                                                         #this is the main method
    rospy.init_node('object_detection', anonymous=True)                                 #for initializing the node with name object_detection
    depth_sub = rospy.Subscriber("/camera/depth/image_raw2", Image, callback_depth)     #for setting the depth image subscriber
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)           #for setting the color image subscriber
    try:
        rospy.spin()                    #this is to keep the node alive
    except KeyboardInterrupt:           #when key interrupt is triggered Shutting down message is displayed and then the windows are shut down
        print("Shutting down")
    cv2.destroyAllWindows()             #for closing the windows

if __name__ == '__main__':
    main(sys.argv)                  #for calling the main method
        
