#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *

send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

def go_fwd():
    command = Twist()
    command.linear.x = 0.15
    send_command(command)
    rospy.sleep(4.0)
    command.linear.x = 0.0
    send_command(command)
    rospy.sleep(1.0)

def turn_left():
    command = Twist()
    command.angular.z = 0.4
    send_command(command)
    rospy.sleep(7.25)
    command.angular.z = 0.0
    send_command(command)
    rospy.sleep(1.0)

def move_square():
    rospy.init_node('MoveSquareNoOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    
    try:
        rospy.sleep(2.0)
        go_fwd()
        go_fwd()
        turn_left()
        go_fwd()
        turn_left()
        go_fwd()
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    move_square()
