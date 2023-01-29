#!/usr/bin/env python3


import sys  # module to handle runtime environment
import rospy  # for use of pytho with ROS
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import geometry_msgs.msg  # for geometry messages
import actionlib  # for importing for providing standard interface e.g. for returning point cloud
import math  # for importing the math library
import time

### GLOBAL VARIABLE ####

linear_vel = 0.0
angular_vel = 0.0

regions = {  # this is to hold the laser values of paricular regions
    'bleft': 0.0,
}

flagEndMotion = False

pose = [-1.0] * 4  # initializing the pose variable

flagDetect = False  # this variable is to check if the tomato is visible in the frame of the camera

motionStage = [False] * 15  # boolean list with all values False is defined
motionStage[0] = True  # the first stage is set to true to start the motion


#######################


def odom_callback(data):  # callback funtion for odom to get the correct position of the bot
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x, y, z, w])[2]]


def moveBot():  # this function is to move the bot with a particular velocity

    '''The stage structure prevents other stages from getting triggered that are not supposed to be triggered ----> this is important as the bot is not moving in a straight line but is moving
    in an environment where the bot has to cross the same locations multiple times so the pose value of the bot will remain same at these locations so to prevent the other stages from getting triggered
    the stage variables are used'''

    global linear_vel
    global angular_vel
    global motionStage  # this is the boolean array defined as global variable
    global pose  # we are accessing the pose of the bot so that we can keep a track of position of the bot
    global flagDetect  # this flag is used to check any tomato is visible in the frame of the camera that is larger than a specific filtering threshold value
    global flagEndMotion
    if motionStage[0] is True:
        if pose[1] >= -0.56:
            motionStage[0] = False  # we are using a step-wise trigger stage where the execution of one stage triggers the execution of the other stage
            motionStage[1] = True
        else:
            rospy.loginfo("Started Run !")
            linear_vel = 0.2  # when we are not to detect anything then the bot is moving with a linear velocity of 2.0
            angular_vel = 0.0
    elif motionStage[1] is True:
        if pose[1] >= 7.22:
            motionStage[1] = False
            motionStage[2] = True
        else:
            linear_vel = 0.2  # if the bot is moving in between the rows but no tomato is visible in the frame of the bot then it will move with a speed of 2.0
            angular_vel = 0.0
            Pcontroller()  # when the bot is moving between the rows the precision in distance is maintained using the P-Controller
    elif motionStage[2] is True:
        if pose[2] <= 0:
            motionStage[2] = False
            motionStage[3] = True
        else:
            if pose[1] <= 8.27:  # pose[1] ---> is the y value of the bot
                linear_vel = 0.2
                angular_vel = 0.0
            elif pose[2] > 0:
                linear_vel = 0.0
                angular_vel = 0.3  # while turning the angular velocity of the bot is 1.7
    elif motionStage[3] is True:
        if pose[0] < -0.647 and pose[2] > -1.6 and pose[2] < -1.5:
            motionStage[3] = False
            motionStage[4] = True
        else:
            if pose[0] >= -0.647:
                linear_vel = 0.2
                angular_vel = 0.0
            elif pose[2] < -1.6 or pose[2] > -1.5:
                linear_vel = 0.0
                angular_vel = 0.3
    elif motionStage[4] is True:
        if pose[1] < 7.7:
            motionStage[4] = False
            motionStage[5] = True
        else:
            linear_vel = 0.2
            angular_vel = 0.0
    elif motionStage[5] is True:
        if pose[1] < 0.0:
            motionStage[5] = False
            motionStage[6] = True
        else:
            linear_vel = 0.2
            angular_vel = 0.0
            Pcontroller()
    elif motionStage[6] is True:
        if pose[1] < -1.3 and pose[2] > 0:
            motionStage[6] = False
            motionStage[7] = True
        else:
            if pose[1] > -1.3 and pose[2] < -1.0:
                linear_vel = 0.2
                angular_vel = 0.0
            elif pose[2] < 0:
                linear_vel = 0.0
                angular_vel = 0.3
    elif motionStage[7] is True:
        if pose[0] > 2.3:
            motionStage[7] = False
            motionStage[8] = True
        else:
            if pose[0] <= 2.3:
                linear_vel = 0.2
                angular_vel = 0.0
    elif motionStage[8] is True:
        if pose[2] > 1.56:
            motionStage[8] = False
            motionStage[9] = True
        else:
            linear_vel = 0.0
            angular_vel = 0.3
    elif motionStage[9] is True:
        if pose[1] >= 7.22:
            motionStage[9] = False
            motionStage[10] = True
        else:
            if pose[1] < -0.56:
                linear_vel = 0.2
                angular_vel = 0.0
            else:
                linear_vel = 0.2
                angular_vel = 0.0
                Pcontroller()
    elif motionStage[10] is True:
        if pose[2] < 0:
            motionStage[10] = False
            motionStage[11] = True
        else:
            if pose[1] <= 8.27:
                linear_vel = 0.2
                angular_vel = 0.0
            elif pose[2] > 0:
                linear_vel = 0.0
                angular_vel = 0.3
    elif motionStage[11] is True:
        if pose[2] > -1.6 and pose[2] < -1.4:
            motionStage[11] = False
            motionStage[12] = True
        else:
            if pose[0] >= 0.97:
                linear_vel = 0.2
                angular_vel = 0.0
            elif pose[2] > -1.4 or pose[2] < -1.6:
                linear_vel = 0.0
                angular_vel = 0.3
    elif motionStage[12] is True:
        if pose[1] < 7.7:
            motionStage[12] = False
            motionStage[13] = True
        else:
            linear_vel = 0.2
            angular_vel = 0.0
    elif motionStage[13] is True:
        if pose[1] < 0.0:
            motionStage[13] = False
            motionStage[14] = True
        else:
            linear_vel = 0.2
            angular_vel = 0.0
            Pcontroller()
    elif motionStage[14] is True:
        if pose[1] < -1.39:
            motionStage[14] = False
            flagEndMotion = True
        else:
            if pose[1] >= -1.39:
                linear_vel = 0.2
                angular_vel = 0.0
    else:
        stopMotion()  # this will stop the bot when it returns to the start after all execution


def stopMotion():  # this function is to set the value of the linear_vel and angular_vel to 0 to stop the bot
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.0


def laser_callback(msg):
    global regions
    global bleft
    bleft = min(msg.ranges[500:531])
    if bleft == 0:                          #change this to if < 0.1m then to 50
        bleft = 50
    if bright == 0:
        bright = 50
    regions = {
        'bleft': min(bleft, 1.5),
    }


def Pcontroller():  # function for the proportional controller --> this has been used so that the bot is able to correct its path if it gets deflected form its path
    global angular_vel
    global regions
    global linear_vel
    kp = 1.5  # the correction factor for the P-Controller is kept 10 so as to attain faster correction
    angular_vel = kp * (regions['bleft'] - 0.60)  # when the left value of the distance as calculated by the laser is 0.65 then the angular_vel is 0 and any deflection from this value will set a certain angular_vel to correct that deflection
    if angular_vel > 1:
        angular_vel = 0.5
    if (regions['bleft'] - 0.60) > 0.2 or (regions['bleft'] - 0.60) < -0.2:
        linear_vel = 0.18
    


def main(args):  # this is the main method

    rospy.init_node('object_detection_manipulation',anonymous=True)  # for initializing the node with name object_detection
    rospy.Subscriber('/odom', Odometry, odom_callback)  # setting up the subscriber
    rospy.Subscriber('/scan', LaserScan, laser_callback)  # setting up the subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # setting up the publisher

    velocity_msg = Twist()

    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)
    global linear_vel
    global angular_vel
    global flagEndMotion


    # while pose[1] == 0.0 and pose[0] == 0.0:
    #     linear_vel = 0.2
    #     angular_vel = 0.0
    #     velocity_msg.linear.x = 0
    #     velocity_msg.linear.y = 0
    #     pub.publish(velocity_msg)


    start_time  = time.time()
    seconds = 4
    while not rospy.is_shutdown():  # while loop
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= seconds:
            
            if flagEndMotion is True:  # gets triggered when the bot returns to its start location after completing the motion
                stopMotion()  # to stop the motion when the allignment of the aruco
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                rospy.loginfo('Mission Accomplished!')
                break

            moveBot()  # if the TF of the bot is not available then the bot has to keep moving
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)  # publishing the value to the bot
            rospy.sleep(0.1)


if __name__ == '__main__':
    main(sys.argv)  # for calling the main method
