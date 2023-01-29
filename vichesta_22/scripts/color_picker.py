#!/usr/bin/env python3


import cv2
import numpy as np
import sys
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


cv_image = None
h_min = 0


def empty(a):
    pass



cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",250,250)
cv2.createTrackbar("Hue Max","TrackBars",0,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
cv2.createTrackbar("Sat Max","TrackBars",0,255,empty)
cv2.createTrackbar("Val Min","TrackBars",0,255,empty)
cv2.createTrackbar("Val Max","TrackBars",0,255,empty)


    



def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    while not rospy.is_shutdown():
        data = rospy.wait_for_message("/camera/color/image_raw2", Image)
        try:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = frame


            '''uncoment to view the visual of detection'''
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


        imgHsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
        v_min = cv2.getTrackbarPos("Val Min","TrackBars")
        v_max = cv2.getTrackbarPos("Val Max","TrackBars")


        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])
        # print(type(np.array({12,131,31})))
        mask = cv2.inRange(imgHsv,lower,upper)
        # print(lower,upper)
        imgResult = cv2.bitwise_and(frame,frame,mask=mask)

        cv2.imshow("HSV",imgHsv)
        cv2.imshow("ImageResult",imgResult)
        cv2.imshow("Mask",mask)
        cv2.waitKey(1)

if __name__ == '__main__':
    main(sys.argv)


