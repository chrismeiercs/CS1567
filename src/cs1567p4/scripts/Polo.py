#!/usr/bin/python
import rospy
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent

constant_command_service = None
send_command             = rospy.ServiceProxy('constant_command', ConstantCommand)

def initialize_commands():
    global constant_command_service
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.init_node('polonode', anonymous=True)
    rospy.wait_for_service('constant_command')
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    
if __name__ == "__main__":   
    try: 
        initialize_commands()

    except rospy.ROSInterruptException: pass