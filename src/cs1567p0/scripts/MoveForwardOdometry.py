#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *
from kobuki_msgs.msg import BumperEvent

bumper = 0;

def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
 

def bumper_callback(data):
    print "HEY!"
    print data

def initialize():
    global bumper
    bumper = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveForwardOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    

if __name__ == "__main__":
    initialize()

