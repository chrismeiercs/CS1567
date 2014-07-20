#!/usr/bin/python
import rospy
import math
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

#Globals
constant_command_service = None
send_command             = rospy.ServiceProxy('constant_command', ConstantCommand)
myLocation               = {'x' : 0.0, 'y' : 0.0}
poloLocations            = [] #array of dictionaries!
closestPoloLocation      = {'x' : 0.0, 'y' : 0.0} 
desiredXLocation         = 0.0
desiredAngle             = 0.0
GAMEOVER                 = False
STATE                    = 0

#Constants
LINEAR_SPEED      = 0.15 #TODO determine good speed for marco, compared to polo
ANGULAR_SPEED     = 0.45 #TODO determine good speed for marco, compared to polo
LINEAR_THRESHOLD  = 0.05
ANGULAR_THRESHOLD = 0.05

#Functions
''' 
@description:
    Linear movement forward, stopping when myx == xLocation
@param: 
    double  xLocation : the x location to stop at 
'''
def goToX(xLocation):

''' 
@description:
    Angular movement left or right for a set distance
@param: 
    string direction : 'left' or 'right' 
    double  distance : 
'''
def turn(direction, distance):

'''
@description:
    Face the robot directly north
'''
def faceUp():
    global STATE
    STATE = 'faceUp'
    # send turn command depending

''' 
@description:
    Stop all movement
'''
def stop():

'''
@description:
    Calculate the distance between two points
@param:
    dictionary point1 : the first point
    dictionary point2 : the second point
@return:
    double distance in meters
'''
def calculateDistance(point1, point2):
    x1 = point1['x']
    x2 = point2['x']
    y1 = point1['y']
    y2 = point2['y']
    result = math.sqrt( abs(x1-x2)**2 + abs(y1-y2)**2 )
    return result
    
'''
@description:
    Calculate the angle between two points
@param:
    dictionary point1 : the first point
    dictionary point2 : the second point
@return:
    double angle in radians
'''
def calculateAngle(point1, point2):
    x1 = point1['x']
    x2 = point2['x']
    y1 = point1['y']
    y2 = point2['y']
    xside = abs(x1-x2)
    yside = abs(y1-y2)
    result = math.atan(xside/yside)
    return result

'''
@description:
    Assign the closestPoloLocation global variable
    based on the shortest distance of all poloLocations
'''
def assignClosestPoloLocation():
    global closestPoloLocation
    minDistance  = 0.0
    tempDistance = 0.0

    # For every polo-point
    for location in poloLocations:
        # Calculate the distance to it
        tempDistance = calculateDistance(myLocation, location)

        # If minDistance isn't set, set it
        if minDistance == 0.0:
            minDistance = tempDistance

        # If tempDistance < minDistance, assign min and closest
        if tempDistance < minDistance:
            minDistance = tempDistance
            closestPoloLocation = location
    
'''
@description:
    Assign the desiredXLocation global variable
    based on how far the robot should travel
    determined by the x&y of myLocation
    and the closestPoloLocation
'''
def setDesiredXLocation():
    global desiredXLocation
    desiredXLocation = closestPoloLocation['x']

'''
@description:
    Assign the desiredAngle global variable
    based on how far the robot should turn
    determined by the x&y of myLocation
    and the closestPoloLocation
'''
def setDesiredAngle():
    global desiredAngle
    desiredAngle = calculateAngle(myLocation, closestPoloLocation)

'''TODO'''
def callMarco():
    #global STATE = 0
    #stop
    #tell everyone to stop
    #busy wait
    #get array of dictionaries from jump
    #assign poloLocations array
    
'''TODO'''
def bumperCallback(data):
    #if released: return
    #stop
    #tell everyone to stop
    #determine if jump says gameover
    #if gameover: gameover
    #else: execute wall avoidance

'''TODO'''
def odomCallback(data):
    global STATE
    if   STATE == 'waiting' or STATE == 'done':
        return
    elif STATE == 'faceUp':
        # Get current yaw in radians
        euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        yaw = euler[2]
        # Check if current yaw is up
        if ( abs(yaw-0.0) <= ANGULAR_THRESHOLD ):
            stop()
            STATE = 'turn'
    elif STATE == 'turn':
        # Get current yaw in radians
        euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        yaw = euler[2]
        # Check if in desired state
        if ( abs(yaw-desiredAngle) <= ANGULAR_THRESHOLD ):
            stop()
            STATE = 'goToX'
    elif STATE == 'goToX':
        xpos = #Get x pose.pose.position
        if ( abs(xpos-desiredXLocation) <= LINEAR_THRESHOLD ):
            stop()
            STATE = 'done'

    
'''TODO'''
def travel():
    faceUp()
    # note: turn and go
    # are handled by the odomCallback

'''TODO'''
def action():
    callMarco()                 # get list of poloLocations
    assignClosestPoloLocation() # determine closest
    #Now we have the closest Polo
    setDesiredAngle()           # set angle
    setDesiredXLocation()       # set XLocation
    #Now everything is set
    travel()                     # faceUp, turn, go          

def initialize_commands():
    global constant_command_service
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.init_node('marconode', anonymous=True)
    rospy.wait_for_service('constant_command')
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    
if __name__ == "__main__":   
    global STATE
    try: 
        initialize_commands()
        
        STATE = 'done' #init

        while(not GAMEOVER):
            if STATE == 'done':
                STATE = 'waiting'
                action()

    except rospy.ROSInterruptException: pass