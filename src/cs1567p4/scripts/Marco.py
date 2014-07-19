#!/usr/bin/python
import rospy
import math
from cs1567p4.srv import *
from std_srvs.srv import * 
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent

#Globals
constant_command_service = None
send_command             = rospy.ServiceProxy('constant_command', ConstantCommand)
myLocation               = {'x' : 0.0, 'y' : 0.0}
poloLocations            = [] #array of dictionaries!
closestPoloLocation      = {'x' : 0.0, 'y' : 0.0} 
desiredDistance          = 0.0
desiredAngle             = 0.0
desiredFace              = ''
GAMEOVER                 = False

#Constants
LINEAR_SPEED             = 0.15 #TODO determine good speed for marco, compared to polo
ANGULAR_SPEED            = 0.45 #TODO determine good speed for marco, compared to polo

#Functions
''' 
@description:
    Linear movement forward or backward for a set distance
@param: 
    string direction : 'forward' or 'backward' 
    double  distance : 
'''
def go(direction, distance):

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
    Face the robot either directly north or directly south
@param:
    string direction : 'north' or 'south'
'''
def face(direction):

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
    Assign the desiredFace global variable
    based on which way the robot should face
    determined by the Y coordinates of myLocation
    and the closestPoloLocation 
'''
def setDesiredFace():
    global desiredFace
    my_y   = myLocation['y']
    polo_y = closestPoloLocation['y']
    if my_y <= polo_y:
        desiredFace = 'south'
    else:
        desiredFace = 'north'
    
'''
@description:
    Assign the desiredDistance global variable
    based on how far the robot should travel
    determined by the x&y of myLocation
    and the closestPoloLocation
'''
def setDesiredDistance():
    global desiredDistance
    desiredDistance = calculateDistance(myLocation, closestPoloLocation)

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
    #stop
    #tell everyone to stop
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
def action():
    callMarco()                 # get list of poloLocations
    assignClosestPoloLocation() # determine closest
    #Now we have the closest Polo
    setDesiredFace()            # set face north/south
    face(desiredFace)           # execute face north/south
    setDesiredAngle()           # set angle
    setDesiredDistance()        # set distance
    turn(desiredAngle)          # execute turn towards closest 
    go(desiredDistance)         # execute travel towards closest

def initialize_commands():
    global constant_command_service
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.init_node('marconode', anonymous=True)
    rospy.wait_for_service('constant_command')
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)
    
if __name__ == "__main__":   
    try: 
        initialize_commands()

    except rospy.ROSInterruptException: pass