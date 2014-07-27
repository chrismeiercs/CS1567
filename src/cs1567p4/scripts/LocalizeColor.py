#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import *
from cs1567p2.msg import *
import math

robot_mask_list =[[marco],[polo1],[polo2]] 
robot_found = [False, False, False]

robots=None
robots_cord = None
threshold = 20
locpub = None
kinect1pub = None
kinect2pub = None #mid
kinect3pub = None #bot
bot_mask = Image() #3
top_mask = Image() 
mid_mask = Image() #mid
found = None
#LocationList = []

robots_move = False
polo_pub = None 
#raw - lists of x,y,kinect
marco_raw = []
polo1_raw = []
polo2_raw = []
raw_list = [marco_raw, polo1_raw, polo2_raw]
robot_found = [False, False, False]

#locations - lists of x,y after being adjusted for kinect number
marco_location = None
polo1_location = None
polo2_location = None

angle_pub = None
distance_pub = None
closest_polo = None

robot_pos = [None, None, None]
robot_x = [None, None, None]
robot_y = [None, None, None]
robot_kinects = [None, None, None]

real_x = [None, None, None]
real_y = [None, None, None]

real_x_adj = [None, None, None]
real_y_adj = [None, None, None]  


def kinect1_image_callback(message):
    print "Start image callback"
    
    global kinect1pub
    
    robots = None
    avg_x = 0
    avg_y = 0
    pos_count = 0
    mask_byte_array = []
    mask = Image()
    mask.width =  message.width
    mask.height = message.height
    mask.encoding = message.encoding
    mask.is_bigendian = message.is_bigendian
    mask.step = message.step
    
    byte_array = list(message.data)

    for pixel in xrange(message.width*message.height):
        for robot in xrange(len(robot_mask_list)):
            if robot_found[robot] != True:
                blue = robot_mask_list[robot][0]
                green = robot_mask_list[robot][1]
                red = robot_mask_list[robot][2]

                if abs(blue - ord(byte_array[3*index])) < threshold\
                   and abs(blue - ord(byte_array[3*index+1])) < threshold\
                   and abs(red - ord(byte_array[3*index+2])) < threshold:
                    byte_array[3*index+0] = chr(color_mask_list[k][0])
                    byte_array[3*index+1] = chr(color_mask_list[k][1])
                    byte_array[3*index+2] = chr(color_mask_list[k][2])

                    robot_x[robot] += index / 640
                    robot_y[robot] += index % 640
                    robot_pos[robot] += 1

                else:
                    byte_array[3*index+0] = chr(0)
                    byte_array[3*index+1] = chr(0)
                    byte_array[3*index+2] = chr(0)

                
                
          
    for robot_num in xrange(len(robot_found)):
        if robot_pos[robot_num] != 0:
            robot_found[robot_num] = True
            robot_kinects[robot_num] = 1
            robot_x[robot_num] = robot_x[robot_num] / robot_pos[robot_num]
            robot_y[robot_num] = robot_y[robot_num] / robot_pos[robot_num]
            
    
    mask.data = "".join(mask_byte_array)
    kinect1pub.publish(mask)   
    #print "Average x: ", x
    #print "Average y: ", y

def kinect1_cloud_callback(message):#3
    global real_x
    global real_y
    global robot_x
    global robot_y
    global robot_kinects
    locations = []

    for robot_in_cloud in xrange(len(robot_kinects)):
        locations = []
        if robot_kinects[robot_in_cloud] == 1:
            rel_points = [robot_x[robot_in_cloud], robot_y[robot_in_cloud]]
    
            try:
                data_out = pc2.read_points(message, field_names=None, skip_nans=True, uvs=[rel_points])
                i=0
                iteration1 = next(data_out) #format x,y,z,rgba
                
                locations.append(iteration1) #should be robot location list
                while iteration1 != None:
                    iteration1 = next(data_out)
                    
                    locations.append(iteration1) #should be orientation list
                    i=i+1
            except StopIteration:
                real_x[robot_in_cloud] = locations[0][0]
                real_y[robot_in_cloud] = locations[0][1]
                    #print "Real World x: ", locations[0][0]
                    #print "Real World y: ", locations[0][1]
                    
        print "done cloud"

def kinect2_image_callback(message):
    print "Start image callback"
    
    global kinect2pub
    
    robots = None
    avg_x = 0
    avg_y = 0
    pos_count = 0
    mask_byte_array = []
    mask = Image()
    mask.width =  message.width
    mask.height = message.height
    mask.encoding = message.encoding
    mask.is_bigendian = message.is_bigendian
    mask.step = message.step
    
    byte_array = list(message.data)

    for pixel in xrange(message.width*message.height):
        for robot in xrange(len(robot_mask_list)):
            if robot_found[robot] != True:
                blue = robot_mask_list[robot][0]
                green = robot_mask_list[robot][1]
                red = robot_mask_list[robot][2]

                if abs(blue - ord(byte_array[3*index])) < threshold\
                   and abs(blue - ord(byte_array[3*index+1])) < threshold\
                   and abs(red - ord(byte_array[3*index+2])) < threshold:
                    byte_array[3*index+0] = chr(color_mask_list[k][0])
                    byte_array[3*index+1] = chr(color_mask_list[k][1])
                    byte_array[3*index+2] = chr(color_mask_list[k][2])

                    robot_x[robot] += index / 640
                    robot_y[robot] += index % 640
                    robot_pos[robot] += 1

                else:
                    byte_array[3*index+0] = chr(0)
                    byte_array[3*index+1] = chr(0)
                    byte_array[3*index+2] = chr(0)

                
                
          
    for robot_num in xrange(len(robot_found)):
        if robot_pos[robot_num] != 0:
            robot_found[robot_num] = True
            robot_kinects[robot_num] = 2
            robot_x[robot_num] = robot_x[robot_num] / robot_pos[robot_num]
            robot_y[robot_num] = robot_y[robot_num] / robot_pos[robot_num]
            
    
    mask.data = "".join(mask_byte_array)
    kinect1pub.publish(mask)   
    #print "Average x: ", x
    #print "Average y: ", y

def kinect2_cloud_callback(message):#3
    global real_x
    global real_y
    global robot_x
    global robot_y
    global robot_kinects
    locations = []

    for robot_in_cloud in xrange(len(robot_kinects)):
        locations = []
        if robot_kinects[robot_in_cloud] == 2:
            rel_points = [robot_x[robot_in_cloud], robot_y[robot_in_cloud]]
    
            try:
                data_out = pc2.read_points(message, field_names=None, skip_nans=True, uvs=[rel_points])
                i=0
                iteration1 = next(data_out) #format x,y,z,rgba
                
                locations.append(iteration1) #should be robot location list
                while iteration1 != None:
                    iteration1 = next(data_out)
                    
                    locations.append(iteration1) #should be orientation list
                    i=i+1
            except StopIteration:
                real_x[robot_in_cloud] = locations[0][0]
                real_y[robot_in_cloud] = locations[0][1]
                    #print "Real World x: ", locations[0][0]
                    #print "Real World y: ", locations[0][1]
                    
        print "done cloud"

def kinect3_image_callback(message):
    print "Start image callback"
    
    global kinect3pub
    
    robots = None
    avg_x = 0
    avg_y = 0
    pos_count = 0
    mask_byte_array = []
    mask = Image()
    mask.width =  message.width
    mask.height = message.height
    mask.encoding = message.encoding
    mask.is_bigendian = message.is_bigendian
    mask.step = message.step
    
    byte_array = list(message.data)

    for pixel in xrange(message.width*message.height):
        for robot in xrange(len(robot_mask_list)):
            if robot_found[robot] != True:
                blue = robot_mask_list[robot][0]
                green = robot_mask_list[robot][1]
                red = robot_mask_list[robot][2]

                if abs(blue - ord(byte_array[3*index])) < threshold\
                   and abs(blue - ord(byte_array[3*index+1])) < threshold\
                   and abs(red - ord(byte_array[3*index+2])) < threshold:
                    byte_array[3*index+0] = chr(color_mask_list[k][0])
                    byte_array[3*index+1] = chr(color_mask_list[k][1])
                    byte_array[3*index+2] = chr(color_mask_list[k][2])

                    robot_x[robot] += index / 640
                    robot_y[robot] += index % 640
                    robot_pos[robot] += 1

                else:
                    byte_array[3*index+0] = chr(0)
                    byte_array[3*index+1] = chr(0)
                    byte_array[3*index+2] = chr(0)

                
                
          
    for robot_num in xrange(len(robot_found)):
        if robot_pos[robot_num] != 0:
            robot_found[robot_num] = True
            robot_kinects[robot_num] = 3
            robot_x[robot_num] = robot_x[robot_num] / robot_pos[robot_num]
            robot_y[robot_num] = robot_y[robot_num] / robot_pos[robot_num]
            
    
    mask.data = "".join(mask_byte_array)
    kinect1pub.publish(mask)   
    #print "Average x: ", x
    #print "Average y: ", y

def kinect3_cloud_callback(message):#3
    global real_x
    global real_y
    global robot_x
    global robot_y
    global robot_kinects
    locations = []

    for robot_in_cloud in xrange(len(robot_kinects)):
        locations = []
        if robot_kinects[robot_in_cloud] == 3:
            rel_points = [robot_x[robot_in_cloud], robot_y[robot_in_cloud]]
    
            try:
                data_out = pc2.read_points(message, field_names=None, skip_nans=True, uvs=[rel_points])
                i=0
                iteration1 = next(data_out) #format x,y,z,rgba
                
                locations.append(iteration1) #should be robot location list
                while iteration1 != None:
                    iteration1 = next(data_out)
                    
                    locations.append(iteration1) #should be orientation list
                    i=i+1
            except StopIteration:
                real_x[robot_in_cloud] = locations[0][0]
                real_y[robot_in_cloud] = locations[0][1]
                    #print "Real World x: ", locations[0][0]
                    #print "Real World y: ", locations[0][1]
                    
        print "done cloud"
		
"""
From Marco
-Check win
-Star All
-Stop All
-Get Closest

To Marco:
-Running?
-Angle and distance

To Polo
-Running?

"""





def adjust_positions(): #adjust for depending on which kinect the robot is in
    global real_x
    global real_y
    global robot_kinects

    global marco_location
    global polo1_location
    global polo2_location
   
    kinect_y_offset = 1.030713438987732

    for robot_player in xrange(len(robot_kinects)):
        kinect = robot_kinects[robot_player]
        x = real_x[robot_player]
        y = real_y[robot_player]
        if kinect == 3:
            y += kinect_y_offset
        elif kinect == 2:
            y += 2*kinect_y_offset
        else:
            yield
        

        if robot_player == 0: #marco
            marco_location = [x,y]
        elif robot_player == 1: #polo1
            polo1_location = [x,y]
        else: #polo2
            polo2_location = [x,y]

   
"""
Actions:
From Marco:
0 - start movements
1 - stop movements
2 - get distance to closest polo
3 - get angle to closest polo
4 - check for win

To Marco:
0 - no action taken
1 - action completed
"""   
def marcoAction(message):
    if(message.data == 0): #start all
        playingGame(True)
        action = 1
    
    elif(message.data == 1): #stop all
        playingGame(False)
        action = 1
    
    elif(message.data == 2): #get distance to closest polo
        getDistance()
        action = 1
    
    elif(message.data == 3): #check win within 650 mm
        getAngle()
        action = 1   
    
    elif(message.data == 4): #check win within 650 mm
        checkWin()
        action = 1   
    else:
        action = 0
    
    action_pub.publish(action)


def playingGame(robots_play): #publish
    global robots_move
    robots_move = robots_play

def getAngle():
    
    global closest_polo
    global marco_location
    global polo1_location
    global polo2_location

    marco_x = marco_location[0]
    marco_y = marco_location[1]
    
    if(closest_polo == 0): #polo 1
        polo_x = polo1_location[0]
        polo_y = polo1_location[1]
    
    else: #polo 2
        polo_x = polo2_location[0]
        polo_y = polo2_location[1]

    delta_x = marco_x - polo_x
    delta_y = marco_y - polo_y
    angle_pub.publish(math.atan2(delta_x, delta_y))


def getDistance():
    polo_distances = getPoloDistances()
    closest_polo_distance = min(polo_distances)
    global closest_polo
    closest_polo = polo_distances.index(closest_polo_distance)
    distance_pub.publish(closest_polo_distance)


def getPoloDistances(self):
    adjust_positions()
    global marco_location #x,y list
    global polo1_location #x,y list
    global polo2_location #x,y list
    
    polo1_x = marco_location[0] - polo1_location[0]
    polo1_y = marco_location[1] - polo1_location[1]
    polo1_distance = math.hypot(polo1_x,polo1_y)

    polo2_x = marco_location[0] - polo2_location[0]
    polo2_y = marco_location[1] - polo2_location[1]
    polo2_distance = math.hypot(polo2_x,polo2_y)
    
    return [polo1_distance, polo2_distance]


def checkWin(): #publish
    polo_distance = getPoloDistances()
 
    if polo_distance[0] < .650 or polo_distance[1] < .650:
        win = True
    else:              #not win
        win = False
    
    return win #or as int: int(win)

def poloAction(message):
    action_pub.publish(int(robotsMove))

		
def initialize():
    global kinect1pub
    global kinect2pub
    global kinect3pub
    global locpub
    #global LocationList
    rospy.init_node("localize")
    locpub = rospy.Publisher("/walle/location",LocationList) #publish your locations
    kinect1pub = rospy.Publisher("/walle2/mask1",Image) #test your mask
    #kinect2pub = rospy.Publisher("/walle/mask2",Image)
    #kinect3pub = rospy.Publisher("/walle/mask3",Image)
    rospy.Subscriber("/kinect1/rgb/image_color", Image, kinect1_image_callback)
    rospy.Subscriber("/kinect1/depth_registered/points", PointCloud2, kinect1_cloud_callback)
    adjust_positions()
    #rospy.Subscriber("/kinect2/rgb/image_color", Image, mid_image_callback)
    #rospy.Subscriber("/kinect2/depth_registered/points", PointCloud2, mid_cloud_callback)
    #if (not found): #if the full robot is found in the image, don't look in the other
    #rospy.Subscriber("/kinect3/rgb/image_color", Image, bot_image_callback)
    #rospy.Subscriber("/kinect3/depth_registered/points", PointCloud2, bot_cloud_callback)
    rospy.spin()

if __name__ == "__main__":
    initialize()
