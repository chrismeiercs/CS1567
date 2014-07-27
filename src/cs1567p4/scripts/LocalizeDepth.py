#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import *
from cs1567p2.msg import *
import math

robot_mask_list =[2420,2380,2340] 
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

#locations - lists of x,y after being adjusted for kinect number
marco_location = None
polo1_location = None
polo2_location = None

angle_pub = None
distance_pub = None
closest_polo = None



    


def kinect1_image_callback(message):
    print "Start image callback"
    global robots_cord
    global robot_found
    global kinect1pub
    global raw_list
    robots = None
    avg_x = 0
    avg_y = 0
    pos_count = 0
    mask_byte_array = []
    mask = Image()
    mask.width = 640
    mask.height = 480
    mask.encoding = "bgr8"
    mask.is_bigendian = 0
    mask.step = 1920
    #returna depth as 16-bit unsigned integers (units: mm)
    byte_array = list(message.data)

    height_threshold = 30 #30mm -> 3cm -> ~1 inch
    desired_height = 2420
    #distance = room_height - height_from_floor
    #room height: 2550 mm
    #chair height: 190 mm -> 2360 mm
    #robot height: 130 mm -> 2420 mm
    mask_byte_array=[]
    for pixel in xrange(0,(message.width*message.height)*2,2):
        height_lower_byte = ord(byte_array[pixel])
        height_upper_byte = ord(byte_array[pixel+1]) << 8 #upper byte is second since little endian
        object_height = height_upper_byte + height_lower_byte
        if abs(desired_height-object_height) <= height_threshold:
            mask_byte_array.append(chr(255))
            mask_byte_array.append(chr(255))
            mask_byte_array.append(chr(255))

            avg_x += (pixel/2) /640#x = pixel / 640
            avg_y += (pixel/2) % 640#y = pixel % 640
            pos_count += 1
        else:
            mask_byte_array.append(chr(0))
            mask_byte_array.append(chr(0)) 
            mask_byte_array.append(chr(0))             
        
       
    x = avg_x / pos_count
    y = avg_y / pos_count
    
          
        
    
    mask.data = "".join(mask_byte_array)
    kinect1pub.publish(mask)   
    #print "Average x: ", x
    #print "Average y: ", y

def kinect1_cloud_callback(message):#3
    global robots_cord
    global raw_list
    locations = []
    try:
        data_out = pc2.read_points(message, field_names=None, skip_nans=True, uvs=[])
        i=0
        iteration1 = next(data_out) #format x,y,z,rgba
        
        locations.append(iteration1) #should be robot location list
        while iteration1 != None:
            iteration1 = next(data_out)
	    
            locations.append(iteration1) #should be orientation list
            i=i+1
    except StopIteration:
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
    global marco_raw
    global polo1_raw
    global polo2_raw

    global marco_location
    global polo1_location
    global polo2_location
   
    kinect_y_offset = 1.030713438987732

    robot_info = [marco_raw, polo1_raw, polo2_raw]
   
    for robot in robot_info:
        x = robot[0]
        y = robot[1]
        kinect = robot[2]
      
        if kinect == 1:
            yield

        elif kinect == 2:
            #yield
            y = y + (2*kinect_y_offset)
        elif kinect == 3:
            #yield
            y = y + kinect_y_offset

        if robot_info.index(robot) == 0: #marco
            marco_location = [x,y]
            print "Marco: ", marco_location
        elif robot_info.index(robot) == 1: #polo1
            polo1_location = [x,y]
            print "Polo1: ", polo1_location
        elif robot_info.index(robot) == 2: #polo2
            polo2_location = [x,y]
            print "Polo2: ", polo2_location
   
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
 
    if polo_distance[0] < 650 or polo_distance[1] < 650:
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
    rospy.Subscriber("/kinect1/depth_registered/image_raw", Image, kinect1_image_callback)
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