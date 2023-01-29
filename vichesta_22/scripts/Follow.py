#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
gState = 0
gRegions = {"right" : 10,
        "front_right" : 10,
        "front" : 10,
        "front_left" : 10,
        "left" : 10,}

def LaserScanProcess(msg):
    global gRegions
    # print(len(msg.ranges))
    gRegions = {
       "right" : min(min(msg.ranges[0:143]),10),
        "front_right" : min(min(msg.ranges[144:287]),10),
        "front" : min(min(msg.ranges[288:431]),10),
        "front_left" : min(min(msg.ranges[432:575]),10),
        "left" : min(min(msg.ranges[576:720]),10),
    }
def change_state(a):
    global gState
    gState=a



def find_wall():
    msg = Twist()
    msg.linear.x = 0.3
    msg.angular.z = -0.1
    return msg

def turn_left():
    global gRegions
    msg = Twist()
    msg.angular.z = 1
    return msg

def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def stop():
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x=0
    return msg

def main():
    global pub, gRegions
    global gDesiredPosition, gState
    rospy.init_node('follow', anonymous=True)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    laser_sub = rospy.Subscriber("/ebot/laser/scan", LaserScan , LaserScanProcess)
    # tf_sub = rospy.Subscriber("/result", String, TfCallback)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        print(gRegions)
        move(gRegions)
        if gState == 0:
            msg = find_wall()
        elif gState == 1:
            msg = turn_left()
            # rospy.sleep(1.5)
        elif gState == 2:
            msg = follow_the_wall()
            pass
        elif gState == 3:
            msg = stop()
        else:
            rospy.logerr("Unknown State")
            pass
        pub.publish(msg)
        rate.sleep()


# Avoid obstacles on the way
def move(regions):
    global gRegions
    linear_x = 0
    angular_z = 0

    d = 1
    if regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d:
        state_description =  "case 1 - Nothing"
        change_state(0)
    elif regions["right"] <d-0.3:
        state_description =  "case x - Right"
        change_state(1)
    elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]>d:
        state_description =  "case 2 - Front"
        change_state(1)
    elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]<d:
        state_description =  "case 3 - Front_Right"
        change_state(0)
    elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]>d:
        state_description =  "case 4 - Front_Left"
        change_state(2)
    elif regions["front"] < d and regions["front_left"]>d and regions["right"]<d:
        state_description =  "case 5 - Front and Right"
        change_state(1)
    elif regions["front"] < d and regions["front_left"]<d and regions["right"]>d:
        state_description =  "case 6 - Front and Left"
        change_state(1)
    elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]<d:
        state_description =  "case 7 - All"
        change_state(1)
    elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]<d:
        state_description =  "case 8 - Front_Right and Front_Left"
        change_state(0)
    else:
        state_description = "Unknow  state"
        print("In Else")
    print(state_description)


if __name__ == '__main__':
    main()