"""csci3302_lab4 controller."""
# Copyright (2025) University of Colorado Boulder
# CSCI 3302: Introduction to Robotics

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor

state = "line_follower" # Change this to anything else to stay in place to test coordinate transform functions

LIDAR_SENSOR_MAX_RANGE = 3 # Meters
LIDAR_ANGLE_BINS = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708 # 90 degrees, 1.5708 radians


# These are your pose values that you will update by solving the odometry equations
pose_x = 0.197
pose_y = 0.678
pose_theta = -np.pi


# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot=Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Initialize the Display    
display = robot.getDevice("display")

# get and enable lidar 
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()


# ##### DO NOT MODIFY ANY CODE ABOVE THIS #####

# ##### Part 1: Setup Data structures
# #
# Create an empty list for your lidar sensor readings here,
# as well as an array that contains the angles of each ray 
# in radians. The total field of view is LIDAR_ANGLE_RANGE,
# and there are LIDAR_ANGLE_BINS. An easy way to generate the
# array that contains all the angles is to use linspace from
# the numpy package.

lidar_readings = [0.0] * LIDAR_ANGLE_BINS  
delta_theta = LIDAR_ANGLE_RANGE / (LIDAR_ANGLE_BINS - 1)

lidar_offsets = np.array([(i - 10) * delta_theta for i in range(LIDAR_ANGLE_BINS)])

# #### End of Part 1 #####
 
###### use def functions ######
def world_to_display(wx, wy):
        try:
            dx = int(wx * 300) 
            if dx > 299:
                dx = 299
            dy = int((wy) * 300)
            if dy > 299:
                dy = 299
        except:
            dx = 0
            dy = 0
        return dx, dy
    

def local_to_global(local_x, local_y, robot_x, robot_y, robot_theta):
    global_x = robot_x + local_x * math.sin(robot_theta) + local_y * -math.cos(robot_theta)
    global_y = robot_y + local_x * math.cos(robot_theta) + local_y * math.sin(robot_theta)

    return global_x, global_y


def convert_to_display(local_coords):
    lidar_to_gloabl = []
    
    for indx in range(21):
        dis = lidar_sensor_readings[indx]
        angle = lidar_offsets[indx]
        coord = (dis * math.cos(angle), dis * math.sin(angle))
        res = local_to_global(coord[0], coord[1], pose_x, pose_y, pose_theta) # too global coords
        res = world_to_display(res[0], res[1]) # from global to display coords
        lidar_to_gloabl.append(res)
    return lidar_to_gloabl
    
        
# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:     
    
    # #####################################################
                    # Sensing                           #
    # #####################################################

    # Read ground sensors
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read Lidar           
    lidar_sensor_readings = lidar.getRangeImage() # rhos
    # print(lidar_sensor_readings)

    
    # ##### Part 2: Turn world coordinates into map coordinates
    # #
    # Come up with a way to turn the robot pose (in world coordinates)
    # into coordinates on the map. Draw a red dot using display.drawPixel()
    # where the robot moves.
    display.setColor(0xFF0000)  # Red for robot pose
    # ##### Part 2: Convert World Coordinates to Display Coordinates #####
    robot_dx, robot_dy = world_to_display(pose_x, pose_y)
    # Draw Robot Position
    display.drawPixel(robot_dx, robot_dy)
    
    # ##### Part 3: Convert Lidar data into world coordinates
    # #
    # Each Lidar reading has a distance rho and an angle alpha.
    # First compute the corresponding rx and ry of where the lidar
    # hits the object in the robot coordinate system. Then convert
    # rx and ry into world coordinates wx and wy. 
    # The arena is 1x1m2 and its origin is in the top left of the arena. 
    
    
    globals_coords = convert_to_display(lidar_sensor_readings)
    
    # ##### Part 4: Draw the obstacle and free space pixels on the map
    display.setColor(0xFFFFFF)
    for coor in globals_coords:
        display.drawLine(robot_dx, robot_dy, coor[0], coor[1])
        
        
    display.setColor(0x0000FF)
    for coor in globals_coords:
        display.drawPixel(coor[0], coor[1])
    
    display.setColor(0xFF0000)  # Red for robot pose
    # ##### Part 2: Convert World Coordinates to Display Coordinates #####
    robot_dx, robot_dy = world_to_display(pose_x, pose_y)
    # Draw Robot Position
    display.drawPixel(robot_dx, robot_dy)
    
    # DO NOT CHANGE THE FOLLOWING CODE (You might only add code to display robot poses)
    # #####################################################
                    # Robot controller                  #
    # #####################################################

    if state == "line_follower":
            if(gsr[1]<350 and gsr[0]>400 and gsr[2] > 400):
                vL=MAX_SPEED*0.3
                vR=MAX_SPEED*0.3                
            # Checking for Start Line          
            elif(gsr[0]<500 and gsr[1]<500 and gsr[2]<500):
                vL=MAX_SPEED*0.3
                vR=MAX_SPEED*0.3
                # print("Over the line!") # Feel free to uncomment this
                display.imageSave(None,"map.png") 
            elif(gsr[2]<650): # turn right
                vL=0.2*MAX_SPEED
                vR=-0.05*MAX_SPEED
            elif(gsr[0]<650): # turn left
                vL=-0.05*MAX_SPEED
                vR=0.2*MAX_SPEED
             
    else:
        # Stationary State
        vL=0
        vR=0   
    
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    
    # #####################################################
                       # Odometry                       #
    # #####################################################
    
    EPUCK_MAX_WHEEL_SPEED = 0.11695*SIM_TIMESTEP/1000.0 
    dsr=vR/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    dsl=vL/MAX_SPEED*EPUCK_MAX_WHEEL_SPEED
    ds=(dsr+dsl)/2.0
    
    pose_theta += (dsr-dsl)/EPUCK_AXLE_DIAMETER
    pose_y += ds*math.cos(pose_theta)
    pose_x += ds*math.sin(pose_theta)
    
 

    
    # Feel free to uncomment this for debugging
    # #print("X: %f Y: %f Theta: %f " % (pose_x,pose_y,pose_theta))
