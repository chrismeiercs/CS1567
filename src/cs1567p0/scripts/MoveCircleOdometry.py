#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

running = True

def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    global running

    if(running):
        if(data.pose.pose.orientation.z < -0.1 and data.pose.pose.orientation.z > -0.16):
            running = False
            command.linear.x = 0.0
            command.angular.z = 0.0
        else:
            command.linear.x = 0.4
            command.angular.z = 0.8
        send_command(command)

	
def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveCircleOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    
if __name__ == "__main__":
    initialize()

