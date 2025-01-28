from controller import Robot, DistanceSensor, Motor, LightSensor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize distance sensors
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

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

# initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# states
FOLLOW_WALL_LEFT = 0
TURN_RIGHT_1 = 1
TURN_LEFT_1 = 2
FOLLOW_WALL_RIGHT = 3
TURN_AROUND = 4
STOP = 5
FULL_TURN = 6
TURN_RIGHT_2 = 7
TURN_LEFT_2 = 8

#initializing our state to follow follow the left wall
state = FOLLOW_WALL_LEFT

# Main loop:
while robot.step(TIME_STEP) != -1:
    
    ps_values = []
    ls_values = []
    for i in range(8):
        ps_values.append(ps[i].getValue())
        ls_values.append(ls[i].getValue())

    # triggers for state switches
    left_obstacle =  ps_values[5] > 80.0 or ps_values[6] >80.0
    right_obstacle = ps_values[2] > 80.0 or ps_values[1] > 80.0
    front_obstacle = ps_values[7] > 80.0 or ps_values[0] > 80.0
    light_sensed = ls_values[0] == 0 and  ls_values[1] == 0 and ls_values[2] == 0 and ls_values[5] == 0 and ls_values[6] == 0 and ls_values[7] == 0
   
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    if state == FOLLOW_WALL_LEFT:
        if front_obstacle:
            state = TURN_RIGHT_1

        if not left_obstacle:
            state = TURN_LEFT_1

        if light_sensed:
            state = FULL_TURN
            
    elif state == TURN_RIGHT_1:
         left_speed = 0.5 * MAX_SPEED
         right_speed = -0.5 * MAX_SPEED
         
         if not front_obstacle:
            state = FOLLOW_WALL_LEFT

    elif state == TURN_LEFT_1:
         left_speed = 0*MAX_SPEED
         right_speed = 0.5 *MAX_SPEED
         
         if left_obstacle:
             state = FOLLOW_WALL_LEFT

    elif state == FULL_TURN:
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

        if right_obstacle:
            state = FOLLOW_WALL_RIGHT
    
    elif state == TURN_LEFT_2:
        right_speed = 0.5 * MAX_SPEED
        left_speed = -0.5 * MAX_SPEED
         
        if not front_obstacle:
            state = FOLLOW_WALL_RIGHT

    elif state == TURN_RIGHT_2:
        right_speed = 0 *MAX_SPEED
        left_speed = 0.5 *MAX_SPEED
         
        if right_obstacle:
             state = FOLLOW_WALL_RIGHT
    
    elif state == FOLLOW_WALL_RIGHT:
        if front_obstacle:
            state = TURN_LEFT_2

        if not right_obstacle:
            state = TURN_RIGHT_2

        if light_sensed:
            state = STOP
    
    elif state == STOP:
        right_speed = -0.5 * MAX_SPEED
        left_speed = 0.5 * MAX_SPEED

        if left_obstacle and not front_obstacle:
            right_speed = 0
            left_speed = 0
           
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)