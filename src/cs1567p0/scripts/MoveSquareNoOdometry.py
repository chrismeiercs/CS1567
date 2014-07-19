#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *

def move_square():
    rospy.init_node('MoveSquareNoOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    command = Twist()
    
    try:
        send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
        for i in range(4):
		command.linear.x = 0.3
        	response = send_command(command)
        	rospy.sleep(4.0)
        	command.linear.x = 0.0
        	response = send_command(command)

		rospy.sleep(1.0)		

		if (i != 3):
			command.angular.z = 0.4
			response = send_command(command)
			rospy.sleep(7.0)
			command.angular.z = 0.0
			response = send_command(command)
			rospy.sleep(1.0)
        
	print response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    move_square()
