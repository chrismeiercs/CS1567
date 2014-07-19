#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *
from tf.transformations import euler_from_quaternion

running = True

def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    global running

    print(data)

    euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

    print "euler from quaternion ", euler

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

