#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

count = 0
forward = True
next = False
pub = 0
timer = 0
do_continue = True

def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    global count
    global forward 
    global next
    global pub
    global timer
    global do_continue

    if(next):
        timer = timer + 1
        if (timer == 100):
            do_continue = True
            next = False
            timer = 0
        
    if(do_continue and count < 4):
        if(forward):
            if(data.pose.pose.position.x < 0.9):
                command.linear.x = 0.2
            else:
                command.linear.x = 0.0
                next = True
                do_continue = False
                count = count + 1 
        else:
            if(data.pose.pose.orientation.z < .69):
                command.angular.z = 0.4
            else:
                command.angular.z = 0.0
                next = True
                do_continue = False
        if(next):
            forward = not forward
            pub.publish(Empty())

        send_command(command)
    
def initialize():
    global forward
    global count
    global next
    global pub
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveSquareOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while (pub.get_num_connections() < 1):
    	rospy.sleep(0.1)
    pub.publish(Empty())
    rospy.spin()
    forward = True
    count = 0
    next = False


if __name__ == "__main__":
    initialize()

