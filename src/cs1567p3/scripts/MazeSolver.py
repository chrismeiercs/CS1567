#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import * 
from geometry_msgs.msg import Twist

LEFT = 0
RIGHT = 1
UP = 2
DOWN = 3

make_maze_service = None
print_maze_service = None
get_wall_service = None
constant_command_service = None

#my globals
myDirection         = 0
myLocationRow       = 0 #Y
myLocationCol       = 0 #X  
serviceDirectionMap = [UP, RIGHT, DOWN, LEFT] 
TURNLEFT  = -1
TURNRIGHT =  1
send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
counter = 0

def moveForward():
    #first, move foward a half meter
    command = Twist()
    command.linear.x = 0.15
    send_command(command)
    rospy.sleep(4.0)
    command.linear.x = 0.0
    send_command(command)
    rospy.sleep(1.0)
    #finally, remember the new location                   
    setLocation()

def turnLeft():
    #first, spin left
    command = Twist()
    command.angular.z = 0.4
    send_command(command)
    rospy.sleep(7.25)
    command.angular.z = 0.0
    send_command(command)
    rospy.sleep(1.0)
    #finally, remember the new direction                        
    setDirection(TURNLEFT)

def turnRight():
    #first, spin right
    command = Twist()
    command.angular.z = -0.4 #negative four??? TODO
    send_command(command)
    rospy.sleep(7.25)
    command.angular.z = 0.0
    send_command(command)
    rospy.sleep(1.0)
    #finally, remember the new direction
    setDirection(TURNRIGHT)

# Returns if there is a wall in the front
# @return boolean
def isWallFront():
    return myWallService(myDirection)
    
# Returns if there is a wall in to the left
# @return boolean
def isWallLeft():
    return myWallService(theLeft())

# Calls the official wall service with a given direction
# @param  the direction to check (in front or to the left)
# @return boolean
# @private
def myWallService(direction):
    stringReturn = get_wall_service(myLocationCol, myLocationRow, serviceDirection(direction))
    if("1" in str(stringReturn)):
        return True
    return False
  
# Returns the direction associated with the left of the robot
# @return A Direction
# @private
def theLeft():
    result = myDirection - 1
    if(result == -1):
        result = 3
    return result

# Set myDirection after a turn
# @param: TURNLEFT (-1) or TURNRIGHT (+1)
# @private
def setDirection(turn):
    global myDirection 
    myDirection = myDirection + turn
    if(myDirection == -1):
        myDirection = 3
    if(myDirection == 4):
        myDirection = 0

# Set myLocation after a move forward
# @private
def setLocation():
    global myLocationRow
    global myLocationCol
    if(myDirection == 0): #up
        myLocationRow -= 1
    if(myDirection == 2): #down
        myLocationRow += 1
    if(myDirection == 3): #left
        myLocationCol -= 1
    if(myDirection == 1): #right
        myLocationCol += 1

# Map myDirection to the proper direction expected by the service
# @param  A direction
# @return An official service direction
# @private
def serviceDirection(direction):
    return serviceDirectionMap[direction]

def checkForShortcutsToGoal():
    if(myLocationRow==4 and myLocationCol==3):
        if(myDirection==1):
            print("SC1 testing...")
            if(not isWallFront()):
                print("SC1!")
                moveForward()
        if(myDirection==2):
            print("SC2 testing...")
            if(not isWallLeft()):
                print("SC2!")
                turnLeft()
                moveForward()
    if(myLocationRow==3 and myLocationCol==4):
        if(myDirection==2):
            print("SC3 testing...")
            if(not isWallFront()):
                print("SC3!")
                moveForward()
        if(myDirection==1):
            print("SC4 testing...")
            turnRight()
            if(isWallFront()):
                turnLeft()
            else:
                print("SC4!")
                moveForward()

def solve_maze(): #'main' function
    print("Hello! Starting the solver...")
    rospy.sleep(2.0)
    global counter
    while(not(myLocationRow==4 and myLocationCol==4)):
        #counter logic
        if(counter==5):
            print("Reposition me!")
            print("    Row: %d  |  Col: %d" % (myLocationRow, myLocationCol))
            counter = 0
            rospy.sleep(6.0)
        counter+=1
        #maze solving logic
        if(isWallLeft()):
    	    if(isWallFront()):
    	        turnRight()
    	    else:
    	    	moveForward()
    	else:
    	    turnLeft()
    	    moveForward()
    	#shortcut logic
    	checkForShortcutsToGoal()
    return 1

def initialize_commands():
    rospy.init_node('mazesolvernode', anonymous=True)
    rospy.wait_for_service('make_maze')
    rospy.wait_for_service('print_maze')
    rospy.wait_for_service('get_wall')
    rospy.wait_for_service('constant_command')

    global make_maze_service, print_maze_service, get_wall_service
    global constant_command_service

    make_maze_service = rospy.ServiceProxy('make_maze', MakeNewMaze)
    print_maze_service = rospy.ServiceProxy('print_maze', Empty)
    get_wall_service = rospy.ServiceProxy('get_wall', GetMazeWall)
    constant_command_service = rospy.ServiceProxy('constant_command', ConstantCommand)

    make_maze_service(5,5)
    print_maze_service()
    solve_maze()

     
if __name__ == "__main__":   
    try: 
        initialize_commands()
    except rospy.ROSInterruptException: pass

