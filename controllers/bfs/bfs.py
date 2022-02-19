"""bfs controller."""
from controller import Robot, Compass, GPS, Keyboard, Motor
import math
from bfs_target import BFS

XYZA = {
    'X': 0,
    'Y': 1,
    'Z': 2,
    'A': 3
}

Sides = {
    'LEFT': 0,
    'RIGHT': 1
}

my_map = [
    ['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', 'E'],
    ['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'],
    ['0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0'],
    ['0', '1', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0'],
    ['0', '1', '1', '1', '0', '0', '1', '1', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '1', '1', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '1', '1', '0', '1', '1', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1'],
    ['1', '1', '1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1'],
    ['1', '1', '1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '1', '1', '0', '0', '0', '1', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '1', '0', '0', '0', '0'],
    ['0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '1', '1', '0', '0', '1', '0', '0', '0', '0', '1', '1', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '0'],
    ['0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0'],
    ['0', '0', '1', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '1', '1', '0'],
    ['0', '0', '1', '0', '0', '0', '1', '1', '1', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'],
    ['0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0'],
    ['S', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0'],
]

bfs = BFS(my_map)
result = bfs.bfs(my_map)
print("Total step:{}".format(len(result['record'])))
targets = bfs.getTarget(result)


TIME_STEP = 16
TARGET_POINTS_SIZE = len(targets)
DISTANCE_TOLERANCE = 0.01
ANGLE_TOLERANCE = 0.05
MAX_SPEED = 9.42

current_target_index = 0


# create the Robot instance.
robot = Robot()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('+inf'))
right_motor.setPosition(float('+inf'))

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

def robot_set_speed(left_speed, right_speed):
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
def minus(target, pos):
    dir_u = target[0] - pos[0]
    dir_v = target[1] - pos[1]
    return dir_u, dir_v
    
def norm(delta):
    square_res = delta[0]*delta[0] + delta[1]*delta[1]
    return math.sqrt(square_res)
    
def normalize(delta):
    n = norm(delta)
    delta[0] /= n
    delta[1] /= n

def modulus_double(a, m):
    div_i = int(a / m)
    div_d = float(div_i)
    r = a - div_d * m
    if r < 0.0:
        r += m
    return r
    
def angle(v1, v2):
    a = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
    m = 2.0 * math.pi
    return modulus_double(a, m)

def run_autopilot():
    global current_target_index
    speeds = [0.0, 0.0]
    
    pos3D = gps.getValues()
    north3D = compass.getValues()
    
    pos = [pos3D[XYZA['X']], pos3D[XYZA['Z']]]
    north = [north3D[XYZA['X']], north3D[XYZA['Z']]]
    front = [-north[0], north[1]]
    
    dir = []
    dir.append(minus(targets[current_target_index], pos)[0])
    dir.append(minus(targets[current_target_index], pos)[1])
    
    distance = norm(dir)
    normalize(dir)
    
    beta = angle(front, dir) - math.pi
    
    if distance < DISTANCE_TOLERANCE:
        print("Target {} reached\n".format(current_target_index + 1))
        current_target_index += 1
    else:
        if beta > ANGLE_TOLERANCE:
            speeds[Sides['LEFT']] = MAX_SPEED * beta / math.pi
            speeds[Sides['RIGHT']] = -MAX_SPEED * beta / math.pi
        elif beta < -ANGLE_TOLERANCE:
            speeds[Sides['LEFT']] = -MAX_SPEED
            speeds[Sides['RIGHT']] = MAX_SPEED
        else:
            speeds[Sides['LEFT']] = MAX_SPEED - math.pi + beta
            speeds[Sides['RIGHT']] = MAX_SPEED - math.pi - beta
            
    robot_set_speed(speeds[Sides['LEFT']], speeds[Sides['RIGHT']])


while robot.step(timestep) != -1:
    pos3D = gps.getValues()
    if current_target_index >= TARGET_POINTS_SIZE:
        robot_set_speed(0, 0)
        print("Approaching destination......\n")
        break
    else:
        run_autopilot()

# Enter here exit cleanup code.
