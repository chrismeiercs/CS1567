#!/usr/bin/python
import rospy
import math
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

## Constants
LINEAR_SPEED      = 0.15 #TODO determine if good speed for marco, compared to polo
ANGULAR_SPEED     = 0.45 #TODO determine if good speed for marco, compared to polo
LINEAR_THRESHOLD  = 0.05 #TODO determine if good number
ANGULAR_THRESHOLD = 0.05 #TODO determine if good number
BACKUP_DISTANCE   = 5.00 #TODO determine if good number

## Globals
bumper_event    = None
const_cmd_srv   = None
send_command    = rospy.ServiceProxy('constant_command', ConstantCommand)
startPosition   = {'x' : 0.0, 'y' : 0.0}
desiredDistance = 0.0
desiredAngle    = 0.0
GAMEOVER        = False
STATE           = ''

## DEBUG FUNCTIONS
def jump_GetAngle():
    return 2.35 # 3/4's pi # Should turn left on positive and right on negative
def jump_GetDistance():
    return 1.0 # 1 meter
## END DEBUG FUNCTIONS

''' 
======================== TODO ======================== 

- Testing : test BumperEvent and states

- Communicate with JUMP
    - jump_StopAll : stop all poloBots
    - jump_GetAngle : return the angle in radians (-pi < a < pi) to the closest poloBot, assuming marco is facing north
    - jump_GetDistance : return the distance in meters to the closest poloBot
    - jump_StartAll : start all poloBots

- Sound File (marco!)

====================================================== 
'''

##
##
## State Machine Description
##
##
'''
=================  STATE MACHINE  ======================

-> needToCallMarco          : callMarco()

-> needToTurnDesired        : turnDesired()
    -> o_turnDesired            : odometryCallback()

-> needToTravelDesired      : travelDesired()
    -> o_saveStartPosition      : odometryCallback()
        -> o_travelDesired          : odometryCallback()

-> needToFaceNorth          : faceNorth()
    -> o_faceNorth              : odometryCallback

=====================  REPEAT  =========================
'''

##
##
## State Machine Functions
##
##
''' 
=== Main action Loop state machine, sets four main commands in motion
'''
def loop():
    if   STATE == 'needToCallMarco':
        callMarco()
    elif STATE == 'needToTurnDesired':
        turnDesired()
    elif STATE == 'needToTravelDesired':
        travelDesired()
    elif STATE == 'needToFaceNorth':
        faceNorth()

''' 
=== Communicate with Jump, get desiredAngle and desiredDistance to closest polo
'''
def callMarco():
    global desiredAngle
    global desiredDistance
    global STATE
    # Play sound file
    # TODO sound file
    # Everyone stop
    stop()
    # jump_StopAll() # TODO JUMP
    # Set globals
    desiredAngle    = jump_GetAngle() # TODO JUMP
    desiredDistance = jump_GetDistance() # TODO JUMP
    # jump_StartAll() # TODO JUMP
    STATE = 'needToTurnDesired'

'''
=== Turn left or right (depending) until odometryCallback stops it
'''
def turnDesired():
    global STATE
    command = Twist()
    if desiredAngle > 0.0:  # if positive, turn left     #TODO is this the correct way to turn?
        command.angular.z = ANGULAR_SPEED
    else:                   # else       , turn right    #TODO is this the correct way to turn?
        command.angular.z = -ANGULAR_SPEED
    send_command(command)
    STATE = 'o_turnDesired'

''' 
=== Travel forward until odometryCallback stops it
'''
def travelDesired():
    global STATE
    command = Twist()
    command.linear.x = LINEAR_SPEED
    send_command(command)
    STATE = 'o_saveStartPosition'

'''
=== Turn left until odometryCallback stops it
'''
def faceNorth():
    global STATE
    command = Twist()
    command.angular.z = ANGULAR_SPEED
    send_command(command)
    STATE = 'o_faceNorth'

''' 
=== Odometry Callback state machine, determines when to stop the robot and sets next state
'''
def odometryCallback(data):
    if STATE == 'interrupted':
        return

    global STATE

    if   STATE == 'o_faceNorth':
        # Get current yaw in radians
        currentYaw = getCurrentYaw(data)
        # Get to 0.0 w/in threshold
        if abs(currentYaw - 0.0) <= ANGULAR_THRESHOLD:
            stop()
            STATE = 'needToCallMarco'

    elif STATE == 'o_turnDesired':
        # Get current yaw in radians
        currentYaw = getCurrentYaw(data)
        # Get to desiredAngle w/in threshold
        if abs(currentYaw - desiredAngle) <= ANGULAR_THRESHOLD:
            stop()
            STATE = 'needToTravelDesired'

    elif STATE == 'o_saveStartPosition':
        setStartPosition(data)
        STATE = 'o_travelDesired'

    elif STATE == 'o_travelDesired':
        # Get current distance travelled
        distanceTravelled = calculateDistance(startPosition, getCurrentPosition(data)) #global startPosition
        # Get to desiredDistance w/in threshold
        if abs(distanceTravelled - desiredDistance) <= LINEAR_THRESHOLD:
            stop()
            STATE = 'needToFaceNorth'

##
##
## Helper Functions
##
##
''' 
=== Return the currentYaw in radians
'''
def getCurrentYaw(data):
    euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    return euler[2] # Radians

''' 
=== Set the global startPosition dictionary
'''
def setStartPosition(data):
    global startPosition
    startPosition['x'] = data.pose.pose.position.x
    startPosition['y'] = data.pose.pose.position.y

''' 
=== Return the currentPosition in a dictionary
'''
def getCurrentPosition(data):
    currentPosition = {}
    currentPosition['x'] = data.pose.pose.position.x
    currentPosition['y'] = data.pose.pose.position.y
    return currentPosition

''' 
=== Calculate the distance between two "Points" (dictionaries with x and y keys)
'''
def calculateDistance(point1, point2):
    x1 = point1['x']
    x2 = point2['x']
    y1 = point1['y']
    y2 = point2['y']
    result = math.sqrt( abs(x1-x2)**2 + abs(y1-y2)**2 ) # '**' is exponentiation operator (**2 == squared)
    return result

''' 
=== Stop all motion
'''
def stop():
    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0
    send_command(command)

''' 
=== Backup after a BumperEvent
'''
def bumperBackup():
    command = Twist()
    command.linear.x = -LINEAR_SPEED
    send_command(command)
    rospy.sleep(BACKUP_DISTANCE)
    stop()

''' 
=== Bumper Callback for a BumperEvent
'''
def bumperCallback(data): #TODO will this work?
    if(data.state == 0): 
        return #return if state is released
    
    global STATE
    STATE = 'interrupted' #dummy state
    stop()
    bumperBackup()
    STATE = 'needToFaceNorth'

''' 
=== Initialize 
'''
def initialize_commands():
    global const_cmd_srv 
    global bumper_event 
    rospy.Subscriber('/odom', Odometry, odometryCallback)
    rospy.init_node('marconode', anonymous=True)
    rospy.wait_for_service('constant_command')
    const_cmd_srv = rospy.ServiceProxy('constant_command', ConstantCommand)
    bumper_event = rospy.Subscriber('/mobile_base/events/bumper',BumperEvent, bumperCallback)

##
##
## Main
##
##
''' 
=== Main
'''
if __name__ == "__main__":   
    global STATE
    try: 
        initialize_commands()

        STATE = 'needToCallMarco' # init -- start facing north!

        while not GAMEOVER:
            loop()

    except rospy.ROSInterruptException: pass



