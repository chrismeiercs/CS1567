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
GAMEOVER = False
MARCO = True
INTERRUPTED = False
RUNNING = True

#Constants
LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.40

#Functions
def go(direction, duration):
    command = Twist()
    if   direction == "forward"  :
        command.linear.x = LINEAR_SPEED
    elif direction == "backward" :
        command.linear.x = -LINEAR_SPEED
    send_command(command)
    rospy.sleep(duration)
    stop()

def turn(direction, duration):
    command = Twist()
    if   direction == "left"  :
        command.angular.z = ANGULAR_SPEED
    elif direction == "right" :
        command.angular.z = -ANGULAR_SPEED
    send_command(command)
    rospy.sleep(duration)
    stop()

def stop():
    command           = Twist()
    command.linear.x  = 0.0
    command.angular.z = 0.0
    send_command(command)

def arc(direction, duration):
    command = Twist()
    command.linear.x = LINEAR_SPEED
    if not INTERRUPTED:
        send_command(command)
    if   direction == "left"  :
        command.angular.z = ANGULAR_SPEED
    elif direction == "right" :
        command.angular.z = -ANGULAR_SPEED
    if not INTERRUPTED:
        send_command(command)
        rospy.sleep(duration)

def bumperCallback(data):
    global INTERRUPTED
    if(data.state == 0):
        return
    INTERRUPTED = True
    stop()
    
def randomDirection():
    number = random.randrange(0, 2)
    if(number==1):
        return "right"
    else:
        return "left"

def randomDuration():
    return random.randrange(0, 7)

def mainLoop():
    global INTERRUPTED
    while(not GAMEOVER):
        if RUNNING:
            arc(randomDirection(),randomDuration())
            if INTERRUPTED:
                go("backward", 3.0)  
                turn("left", 11.0) 
                INTERRUPTED = False
            

def initialize_commands():
    global constant_command_service
    global bumper_event
    rospy.init_node('polonode', anonymous=True)
    rospy.wait_for_service('constant_command')
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    bumper_event = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, bumperCallback)

if __name__ == "__main__":   
    try: 
        initialize_commands()
        mainLoop()
    except rospy.ROSInterruptException: pass
