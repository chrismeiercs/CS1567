#!/usr/bin/python
import rospy
from cs1567p3.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent

# Globals
bumper_event             = None
constant_command_service = None
send_command             = rospy.ServiceProxy('constant_command', ConstantCommand)
pub = 0
INTERRUPTED = False
CHANGED     = False
STATE       = 1
odomActive = False
count = 0
RUNNING = True

# Constants
LINEAR_SPEED      = 0.10
ANGULAR_SPEED     = 0.40

# Functions
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

''' 
@description:
    Bumper Event occurred
    Set the STATE depending on: 1) the current STATE and 2) the Bumper Event
    (This code is ugly but supports a finite state machine type of algorithm
     Note: these large if/elif's are basically just SWITCH..CASE statements, which are unsupported by Python)
@params:
    Object data : passed by Bumper Event
'''
def bumper_event_interrupt(data):
    global CHANGED
    if (data.state == 0) or CHANGED: #Released
        return

    global STATE
    global INTERRUPTED
    # Stop the robot and notify an interrupt occurred 
    stop()
    result = -1

    #DEBUG
    print "Interrupt. bumper:"
    print data.bumper
    #DEBUG

    if STATE == 1:
        if data.bumper == 0: #LEFT
            # Angle correction
            result = 1.2
        elif data.bumper == 1: #CENTER
            # 90 concave wall
            result = 3
        elif data.bumper == 2: #RIGHT
            result = 3
    elif STATE == 2:
        if data.bumper == 0: #LEFT
            # Wall was on left, yay
            result = 1.5 #GOTO state 1.5 (backup and turn)
        elif data.bumper == 1: #CENTER
            # Probably overcompensated on a 90 convex turn
            result = 3
        elif data.bumper == 2: #RIGHT
            result = 3
    elif STATE == 2.2:
        if data.bumper == 0: #LEFT
            # Just missed the wall, so close.
            result = 1.2
    # Final check to update state
    if result != -1:
        STATE = result
        CHANGED = True
        print "Change state:"
        print STATE
    else:
        print ">>> Invalid State in Interrupt! STATE, bumper"
        print STATE
        print data.bumper
        STATE = -1234.8888

    INTERRUPTED = True

'''
@description:
    Perform actions based on the current STATE
    (This code is ugly but supports a finite state machine type of algorithm
     Note: these large if/elif's are basically just SWITCH..CASE statements, which are unsupported by Python)
'''
def action_loop():
    global STATE
    global INTERRUPTED
    global CHANGED
    global count
    global odomActive
    
    #DEBUG
    print " "
    print "Execute State:"
    print STATE
    #ENDDEBUG

    # Note for all cases: if interrupeted, the state is set by the callback
    if   STATE == 1:                    # Initial move forward
        go("forward", 5.0)
        if INTERRUPTED and not CHANGED:
            STATE = -100
        if not INTERRUPTED:
            # GOTO Check wall left
            STATE = 2
    elif STATE == 1.2:			# Correction angle needed
        go("backward", 1.0)
        turn("right", 1.3)
        STATE = 1
    elif STATE == 1.5:                  # Get away from wall
        go("backward", 1.0)
        turn("right", 2.2) #need to overcompensate here :(
        STATE = 1 

    elif STATE == 2:                    # Check for wall on left
        turn("left", 2.0)
        go("forward", 2.5)
        if INTERRUPTED and not CHANGED:
            STATE = -200
        if not INTERRUPTED:
            STATE = 2.2
    elif STATE == 2.2:			# 90 degree convex wall
        turn("left", 4.3)
        if INTERRUPTED and not CHANGED:
            STATE = -200.2
        if not INTERRUPTED:
            STATE = 1

    elif STATE == 3: 			# 90 degree concave wall (and error correction for state 2.2)
        go("backward", 1.0)
        turn("right", 5.5) #90 degrees
        STATE = 1
        
    else:
        print "INVALID STATE IN Action Loop! -- STATE:"
        print STATE
        print throwerror
        STATE == -9999 #will break out of loop

    # Reinit values:
    INTERRUPTED = False
    CHANGED     = False
    if(count < 20):
    	count = count + 1
    	print count
    elif(not odomActive):
        odomActive = True 


def odom_callback(data):    
    global RUNNING
    if(count == 0):
        print data.pose.pose.position.x
        print data.pose.pose.position.y

    if(not odomActive):
        return
    if(abs(data.pose.pose.position.x) < .3 and abs(data.pose.pose.position.y) < .3):
        RUNNING = False
        print "Close to starting location!"
        print data.pose.pose.position.x
        print data.pose.pose.position.y


def initialize_commands():
    global constant_command_service
    global bumper_event
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.init_node('wallfollownode', anonymous=True)
    rospy.wait_for_service('constant_command')
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    bumper_event = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_event_interrupt)
    print "Initial X, Y:"
    
if __name__ == "__main__":   
    try: 
        initialize_commands()
   
        while STATE > 0 and RUNNING:
            action_loop()	
        stop()

    except rospy.ROSInterruptException: pass

