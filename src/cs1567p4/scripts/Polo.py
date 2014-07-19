#!/usr/bin/python
import rospy
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent

#Globals
constant_command_service = None
send_command             = rospy.ServiceProxy('constant_command', ConstantCommand)
'''TODO globals here'''

#Constants
'''TODO constants here'''


#Functions
''' 
@description:
    Linear movement forward or backward for a set duration
@params: 
    string  direction : 'forward' or 'backward' 
    integer duration  : length of time to sleep
'''
def go(direction, duration):
    command = Twist()
    if   direction == "forward"  :
        command.linear.x = LINEAR_SPEED
    elif direction == "backward" :
        command.linear.x = -LINEAR_SPEED
    send_command(command)
    rospy.sleep(duration)
    stop()

''' 
@description:
    Angular movement left or right for a set duration
@params: 
    string  direction : 'left' or 'right' 
    integer duration  : length of time to sleep
'''
def turn(direction, duration):
    command = Twist()
    if   direction == "left"  :
        command.angular.z = ANGULAR_SPEED
    elif direction == "right" :
        command.angular.z = -ANGULAR_SPEED
    send_command(command)
    rospy.sleep(duration)
    stop()

''' 
@description:
    Stop all movement
'''
def stop():
    command           = Twist()
    command.linear.x  = 0.0
    command.angular.z = 0.0
    send_command(command)

'''TODO'''
def arc(direction, duration):
	
'''TODO'''
def constantGo():
	
'''TODO'''
def callPolo():
	

'''TODO'''
def bumperCallback(data):
	


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