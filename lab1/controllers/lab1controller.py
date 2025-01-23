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

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    front_obstacle = psValues[0] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    
    def setSpeeds(state):
        match state:
            case 0:
            
                return 0.5 * MAX_SPEED, 0.5 * MAX_SPEED
            case 1:
                return 0.5 * MAX_SPEED, -0.5 * MAX_SPEED
            case 2:
                return 0.5 * MAX_SPEED, -0.5 * MAX_SPEED
            case 3:
                return 0.5 * MAX_SPEED, 0.5 * MAX_SPEED
            case 4:
                return -0.5 * MAX_SPEED, 0.5 * MAX_SPEED
            case _:
                return 0, 0
             
    
    
    
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
    
    
    
    
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
