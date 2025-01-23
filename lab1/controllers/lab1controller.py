"""lab1controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, LightSensor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

ls = []
lsNames = [
    'ls0', 'ls1', 'ls2', 'ls3',
    'ls4', 'ls5', 'ls6', 'ls7'
]

state = 0

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

################################ need to tune this stuff prolly without running the whole script #########
#setup tuning variables 
distance_to_wall_side = 5
wall_correction_amount = .03
foward_speed_mult = .3
turn_speed = 1  
turning_radians = 1

# sensor distance checks
def check_front_obstacle(psValues):
    # # detect obstacles
    # right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    # left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    # front_obstacle = psValues[0] > 80.0 or psValues[7] > 80.0
    
    front_dis = 5 ### dunno distancessss need to check ###################
    if psValues[0] > front_dis or psValues[0] > front_dis or psValues[0] > front_dis: # three sensors I think idk
        return True
    return False


# light sensor checks
def check_light_sensor(lsValues):
    light_threshold = 5  ###### light threshold needs to be checkeeeddd ######
    if lsValues[0] > light_threshold:   
        return True
    return False

#turn a certain amount of degrees to the right or clockwise
def turn_robot_degrees(deg: int):
    radians = (deg / 360) * turning_radians
    leftMotor.setPosition(radians)
    rightMotor.setPosition(-radians)
    #sleep until it finishes     IMPORTANT TO IMPLEMENT ############################

#take in a state and return the robot movement
def setSpeeds(state, psValues):
    match state:
        case 0:
            if psValues[left sensor] > distance_to_wall:
                return foward_speed_mult * MAX_SPEED, foward_speed_mult * MAX_SPEED * (1 + wall_correction_amount)
            return foward_speed_mult * MAX_SPEED  * (1 + wall_correction_amount), foward_speed_mult * MAX_SPEED
        case 1:
            turn_robot_degrees(90)
            state = 0
            return 0, 0 
            # return 0.5 * MAX_SPEED, -0.5 * MAX_SPEED
        case 2:
            turn_robot_degrees(180)
            state = 3
            return 0, 0
            # return 0.5 * MAX_SPEED, -0.5 * MAX_SPEED
        case 3:
            if psValues[right sensor] > distance_to_wall:
                return foward_speed_mult * MAX_SPEED * (1 + wall_correction_amount), foward_speed_mult * MAX_SPEED
            return foward_speed_mult * MAX_SPEED, foward_speed_mult * MAX_SPEED * (1 + wall_correction_amount)
        case 4:
            turn_robot_degrees(-90)
            state = 3
            return 0, 0
            # return -0.5 * MAX_SPEED, 0.5 * MAX_SPEED
        case _:
            return 0, 0


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    lsValues = []
    
    for i in range(8):
        psValues.append(ps[i].getValue())
        lsValues.append(ls[i].getValue())
    
    if check_front_obstacle(psValues):
        if state == 0:
            state = 1
        else:
            state = 4
    elif check_light_sensor(lsValues):
        if state == 0:
            state = 2
        else:
            break
    else:
        state = 0
           
    # modify speeds according to obstacles
    # if left_obstacle:
        # turn right
        # leftSpeed  = 0.5 * MAX_SPEED
        # rightSpeed = -0.5 * MAX_SPEED
    # elif right_obstacle:
        # turn left
        # leftSpeed  = -0.5 * MAX_SPEED
        # rightSpeed = 0.5 * MAX_SPEED
    # elif front_obstacle:
    
    leftSpeed, rightSpeed = setSpeeds(state, psValues)
    
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
