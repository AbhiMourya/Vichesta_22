#!/usr/bin/env python3


import cv2                                          #for importing opencv 
import numpy as np                                  #useful in the array operations
import roslib                                       #roslib is an internal library to write tools
import sys                                          #module to handle runtime environment
import rospy                                        #for use of pytho with ROS
from sensor_msgs.msg import Image                   #importing the image type sensor message
from cv_bridge import CvBridge, CvBridgeError       #for importing CvBridge used to convert the ROS format images to opencv format
import tf2_ros                              #imporing tf2
import geometry_msgs.msg                    #for geometry messages
from geometry_msgs.msg import Twist
import tf_conversions                       #importing tf_conversions
import moveit_commander                     #moveit commander for motion planning
import moveit_msgs.msg                      #moveit messages
import actionlib                            #for importing for providing standard interface e.g. for returning point cloud
import math                                 #for importing the math library
import copy
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

redboxColor = [[0, 230, 80, 20, 255, 255]] 
greenboxColor = [[30, 220, 50, 80, 255, 255]]  

class Ur5Moveit(object):

    def __init__(self):
        super(Ur5Moveit, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        self._hand_group = moveit_commander.MoveGroupCommander("gripper")


        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()


        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def display_trajectory(self, plan):
            robot = self.robot
            display_trajectory_publisher = self.display_trajectory_publisher
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
            group = self.group
            group.execute(plan, wait=True)

    def arm_robot(self,state):                          #this function is for taking the arm to predefined pose 
            if state == 1:                                  #When the state of the arm is set to 1 then the arm goes to the Drop_Pose
                self.group.set_named_target("go_to_box1")   #Drop_Pose has been designed to drop the picked up tomatoes effectively in the bucket
                plan3 = self.group.go()                    #For planning and execution of the Drop_Pose ---> go() function handles both the work
            # elif state == 0:                                #State 0 is for the Plant_Perception pose ---> using this pose the TF of all the tomatoes of the plant is collected at time 0
            #     self.group.set_named_target("extend")
            #     plan3 = self.group.go()


    def gripper_robot(self,state):                      #this function is used to control the gripper and it accepts the state variable for setting the state of the gripper
            if state == 1:                                  #state 1 means that the gripper will be in the closed state
                self._hand_group.set_named_target("close")  #this is to set the target to a pre-defined pose of the gripper as close
                plan2 = self._hand_group.go()               #this line plans and executes the instructions given by moveit
            elif state == 0:                                #state 0 means that the gripper will be in the open state
                self._hand_group.set_named_target("open")   #this sets the state to open
                plan2 = self._hand_group.go()

    def plan_cartesian_path(self, scale, valueX, valueY, valueZ):
        move_group = self.group

        # print("Problem###############")
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

def callback_depth(data_depth):             #this callback is for accessing the depth image

    global depth_image                  #this is to update the value in the global depth_image variable

    try:
        bridgeDepth = CvBridge()                #for creating the bridge object
        depth_image = bridgeDepth.imgmsg_to_cv2(data_depth, "32FC1") #for converting the depth image into depth matrix if 32FC1 encoding  ----> each pixel value is stored as one channel floating point with single precision
    except CvBridgeError as e:          #for handling errors
        print(e)

def getContours(img,col):           #this function is used to draw contours in their appropriate places
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
            t.child_frame_id = col+str(counter)           #this is the naming convention where the is given as obj + value of the counter -----> obj1, obj2 etc.

            cv2.putText(imgContour, t.child_frame_id,               #this function is used to put text on the imgContour, the text is the child_frame_id, at the point (midX, midY) 
            (midX, midY), cv2.FONT_HERSHEY_SIMPLEX,         #cv2.FONT_HERSHEY_SIMPLEX ----> is the font used to label the image
            0.5, (255, 0, 0), 2)             #0.5 is the font scale, (255, 0, 0) is for giving blue colour and 2 is the thickness


            t.transform.translation.x = Z          #this is for transforming the world coordinates to the camera frame that is on the arm
            t.transform.translation.y = -X         #this is for transforming the world coordinates to the camera frame that is on the arm
            t.transform.translation.z = -Y         #this is for transforming the world coordinates to the camera frame that is on the arm

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
        for color in greenboxColor:       #this for loop goes through the redboxColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
            lower_g = np.array(color[0:3])        #this is for forming the lower_r range of the HSV colour space ----> i.e. h_min, s_min, v_min
            upper_g = np.array(color[3:6])        #this is for forming the upper_r range of the HSV colour space ----> i.e. h_max, s_max, v_max
            mask_g = cv2.inRange(imgHSV, lower_g, upper_g)            #for mask_ring the image 
            x, y = getContours(mask_g,"Green")
        for color in redboxColor:       #this for loop goes through the redboxColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
            lower_r = np.array(color[0:3])        #this is for forming the lower_r range of the HSV colour space ----> i.e. h_min, s_min, v_min
            upper_r = np.array(color[3:6])        #this is for forming the upper_r range of the HSV colour space ----> i.e. h_max, s_max, v_max
            mask_r = cv2.inRange(imgHSV, lower_r, upper_r)            #for mask_ring the image 
            x, y = getContours(mask_r,"Red")   
    except CvBridgeError as e:          #for handling the error
        print(e)



def main(args):                                                                         #this is the main method

    rospy.init_node('object_detection_manipulation', anonymous=True)                                 #for initializing the node with name object_detection
    tfBuffer = tf2_ros.Buffer()  
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)                                                       #for setting the buffer
    listener = tf2_ros.TransformListener(tfBuffer)                                      #for using the TF listener
    depth_sub = rospy.Subscriber("/camera/depth/image_raw2", Image, callback_depth)     #for setting the depth image subscriber
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)           #for setting the color image subscriber
    
    ur5 = Ur5Moveit()
    print("Here")
    ur5_pose_1 = geometry_msgs.msg.Pose()
    while not rospy.is_shutdown():                      #while loop
        msg=Twist()
        msg.linear.x=1
        pub.publish(msg)
        rospy.sleep(1)
        msg.linear.x=0
        pub.publish(msg)
        ur5.arm_robot(1)
        rospy.sleep(0.1)
        try:                                                                            #try to handle exceptions
            trans = tfBuffer.lookup_transform('base_link', 'obj0', rospy.Time(0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        base_link_Position_X = 0.36                  #this is the x position of the base_link ----> our reference point
        base_link_Position_Y = 0                     #this is the y position of the base_link ----> our reference point
        base_link_Position_Z = 0.36 

        X=trans.transform.translation.x
        Y=trans.transform.translation.y
        Z=trans.transform.translation.z

        ur5_pose_1.position.x = base_link_Position_X + X
        ur5_pose_1.position.y = base_link_Position_Y + Y
        ur5_pose_1.position.z = base_link_Position_Z + Z
        print("Here1")
        cartesian_plan, fraction = ur5.plan_cartesian_path(1, ur5_pose_1.position.x, ur5_pose_1.position.y, ur5_pose_1.position.z)
        ur5.display_trajectory(cartesian_plan)
        ur5.execute_plan(cartesian_plan)


        ur5.gripper_robot(1)                #state 1 of the gripper is to close the gripper
        rospy.sleep(0.1)

        ur5.arm_robot(1)                    #to bring the arm to the bucket
        rospy.sleep(0.1)
        ur5.gripper_robot(0)                #to open the gripper
        rospy.sleep(0.1)
        break
    del ur5             #to delete the ur5 object

if __name__ == '__main__':
    main(sys.argv) 