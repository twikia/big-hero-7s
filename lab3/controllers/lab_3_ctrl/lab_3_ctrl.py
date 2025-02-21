"""csci3302_lab3 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor # type: ignore
import numpy as np

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28
EPUCK_WHEEL_RADIUS = 0.0205

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
leftMax = leftMotor.getMaxVelocity()
rightMax = rightMotor.getMaxVelocity()   

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
# x, y, theta stored in array of tuples
             
waypoints = [
             (-0.304705, -0.414838, 0),
             (0.325295, -0.414838, math.pi/2),
             (0.325295, -0.244838, (3*math.pi)/4),
             (.045295, -0.014838, math.pi/4),
             (0.345295, 0.265162, (3*math.pi)/4),
             (0.125295, 0.425162, math.pi),
             (-0.304705, 0.425162, 0),
             (-.304705, .285162,(3*math.pi)/2),
             (-.194705, .285162, 0),
             (-0.224705, -0.004838, (3*math.pi)/2),
             (-0.304705, -.004838, math.pi),
             (-0.304705, -0.194838, 3*math.pi/2)
             ]

# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
# marker = robot.getFromDef("marker").getField("translation")


# our state variables / other random stuff we might need
step = 0
is_proportional_controller = False
is_proportional_feedback_controller_state = False
state = 'not proportional'
elapsed_time = 0 


# FOR 3.2
def inverse_wheel_kinematics(distance, delta_theta, delta_time=SIM_TIMESTEP / 1000.0, axle_diameter=EPUCK_AXLE_DIAMETER):
    """takes in the amount to travel then gives the rotations we need"""
    if delta_time == 0: return 0, 0
    
    # reversed the equations we had from last lab 2 for odometry
    v_linear = distance / delta_time  
    vR = (v_linear/EPUCK_WHEEL_RADIUS) - (axle_diameter*delta_theta)/(2*EPUCK_WHEEL_RADIUS)
    vL = (axle_diameter*delta_theta)/(2*EPUCK_WHEEL_RADIUS) + (v_linear/EPUCK_WHEEL_RADIUS)
    return vL, vR

def reach_position(distance_to_goal, is_proportional=True) -> tuple:
    """goes foward until the distance is within the error -> ruturns true is it has reached the goal"""
    global leftMotor, rightMotor, leftMax, rightMax
    
    # tuning variables r here
    err_margin = .01
    foward_speed = .5
    min_foward_speed = .01
    max_foward_speed = .5
    
    if is_proportional:
        portional_gain = 5
        foward_speed = abs(distance_to_goal * portional_gain)
    
    if foward_speed > max_foward_speed:
        foward_speed = max_foward_speed
        
    if foward_speed < min_foward_speed:
        foward_speed = min_foward_speed
        
    if not (distance_to_goal < err_margin and distance_to_goal > -err_margin):
        return (leftMax * foward_speed, rightMax * foward_speed)

    return None

def turn_to_goal(ang_to_goal: float, is_proportional=True) -> tuple:
    """ takes in the angle and turns if not facing -> returns true if it is facing the goal otherwise returns false"""
    
    global leftMotor, rightMotor, leftMax, rightMax
    
    # tuning variables r here
    err_margin = .01
    turn_speed = .2 # default value will change this
    min_turn_speed = .01
    max_turn_speed = .25
    portional_gain = .3
    
    # this part does makes it turn faster the further it is
    if is_proportional:
        turn_speed = abs(portional_gain * ang_to_goal)
    
    if turn_speed > max_turn_speed:
        turn_speed = max_turn_speed
        
    if turn_speed < min_turn_speed:
        turn_speed = min_turn_speed

    
    if not (ang_to_goal < err_margin and ang_to_goal > -err_margin):
        if ang_to_goal > 0:
            return (-leftMax * turn_speed, rightMax * turn_speed)
        return (leftMax * turn_speed, -rightMax * turn_speed)
        
    return None


# Main Control Loop:
def main():
    global step, index, gsr, elapsed_time
    
    # local vars that don't need to be global:
    pose_x = 0
    pose_y = 0
    pose_theta = 0
    
    
    while robot.step(SIM_TIMESTEP) != -1:
        
        #########################################################
        # Previous code for line following, for testing
        #########################################################
        # for i, gs in enumerate(ground_sensors):
        #     gsr[i] = gs.getValue()
            
        # center_sensor = gsr[1] < 700
        # left_sensor = gsr[0] < 700
        # right_sensor = gsr[2] < 700

        # marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])

        # Read pose_x, pose_y, pose_theta from gps and compass
        pose_x = gps.getValues()[0]
        pose_y = gps.getValues()[1]
        pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
        
        # get the times:
        elapsed_time += SIM_TIMESTEP / 1000.0
        delta_time = SIM_TIMESTEP / 1000.0
        
        # TODO: controller / calculations
        
        # euclidian distance
        goal_pos = (-0.19, 0.125162, 0)
        goal_pos = waypoints[index]
        euc_dis = math.pow(goal_pos[0] - pose_x, 2)
        euc_dis += math.pow(goal_pos[1] - pose_y, 2)
        euc_dis = math.sqrt(euc_dis)
        
        # calculate the angle to goal
        ang_to_goal = math.atan2(goal_pos[1] - pose_y, goal_pos[0] - pose_x)
        ang_to_goal = (ang_to_goal - pose_theta + math.pi) % (2 * math.pi) - math.pi
        # heading_to_goal_heading = goal_pos[2] - pose_theta
        heading_to_goal_heading = (goal_pos[2] - pose_theta + math.pi) % (2 * math.pi) - math.pi

        if state == 'proportional':
           
            # tuning vars:
            forward_err = 1
            rot_err = 0.01
            forward_gain = 0.1
            rot_gain = .5
           
            
            L_dis, R_dis = inverse_wheel_kinematics(euc_dis, ang_to_goal)
            wheel_rot = L_dis - R_dis # right wheel minus left gives positive theta rot
            
            if not (abs(wheel_rot) < rot_err):
                
             
                res = (-wheel_rot * rot_gain, wheel_rot * rot_gain)
                
            else:
               
                res = (L_dis * forward_gain, R_dis * forward_gain)
                
            # set bounds
            if res[0] > 3:
                res = (3, res[1])
            if res[1] > 3:
                res = (res[0], 3)

            if L_dis < forward_err and R_dis < forward_err:
                print('INCREASING INDEX')
                index += 1
                    
            
        else:
            is_proportional_controller = True
            match step:
                case 0:
                    res = turn_to_goal(ang_to_goal, is_proportional_controller)
                    if res is None:
                        res = reach_position(euc_dis, is_proportional_controller)
                        if res is None:
                            step += 1
                            res = (0, 0)
                        
                case 1:
                    res = turn_to_goal(heading_to_goal_heading, is_proportional_controller)
                    if res is None:
                        res = (0, 0)
                        step = 0
                        index += 1 #iterating to next waypoint
                    
                case _:
                    res = (0, 0)
                    
        leftMotor.setVelocity(res[0])
        rightMotor.setVelocity(res[1])
        
        # exit condition here so that it ends
        if index >= len(waypoints):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            exit(0)



if __name__ == "__main__":
    main()
