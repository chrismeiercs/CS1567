#!/usr/bin/python
import rospy
import random
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent

#Globals
bumper_event = None
constant_command_service = None
send_command             = rospy.ServiceProxy('constant_command', ConstantCommand)
'''TODO globals here'''
GAMEOVER = False
MARCO = True
INTERRUPTED = False
RUNNING = True
#Constants
'''TODO constants here'''
LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.40

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
    print "in arc"

    command = Twist()
    command.linear.x = LINEAR_SPEED
    if not INTERRUPTED:
	print "send arc linear"
        send_command(command)

    if   direction == "left"  :
        command.angular.z = ANGULAR_SPEED
    elif direction == "right" :
        command.angular.z = -ANGULAR_SPEED
    
    if not INTERRUPTED:
	print "send arc angular"
        send_command(command)
        rospy.sleep(duration)
    
    
'''TODO'''
#def callPolo():
    

'''TODO'''
def bumperCallback(data):
    global INTERRUPTED
    if(data.state == 0):
        return

    print "INTERRUPTED", data
    INTERRUPTED = True
    stop()
    
    
def randomDirection():
    number = random.randrange(0,2)
    if(number==1):
        return "right"
    else:
        return "left"

def randomDuration():
    return random.randrange(0,7)

def mainLoop():
    global INTERRUPTED
#    constantGo()
    while(not GAMEOVER):
        if RUNNING:           
            print "start of while, calling arc"
            arc(randomDirection(),randomDuration())
            print "arc finished, checking interrupted"
            if INTERRUPTED :
                print "interrupted: go"
                go("backward", 3.0)
                print "interrupted, turn"  
                turn("left", 11) 
                INTERRUPTED = False
                print "finished interruped"
            

def initialize_commands():
    global constant_command_service
    global bumper_event
    rospy.init_node('polonode', anonymous=True)
    rospy.wait_for_service('constant_command')
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    bumper_event = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, bumperCallback)
    mainLoop()
if __name__ == "__main__":   
    try: 
        initialize_commands()

    except rospy.ROSInterruptException: pass
