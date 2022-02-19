"""test_controller controller."""
from controller import Robot, Motor, DistanceSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
TIME_STEP = 64

MAX_SPEED = 6.28

# 初始化距离传感器
# 距离传感器的名字列表
psNames = ['ps0', 'ps1', 'ps2', 'ps3',
            'ps4', 'ps5', 'ps6', 'ps7']
ds_list = [] 
for name in psNames:
    ds = robot.getDevice(name)
    ds.enable(TIME_STEP)
    ds_list.append(ds)

# 初始化电机
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
# 设置电机为速度控制模式
# Motor_setPosition(leftMotor, '+inf')
# Motor_setPosition(rightMotor, '+inf')
leftMotor.setPosition(float('+inf'))
rightMotor.setPosition(float('+inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# 检测左边是否有障碍物
def LeftObstacleDetected(psValues):
    if psValues[0] >500.0 or psValues[1] > 500.0 or psValues[2] > 500.0:
        return True
    else:
        return False

# 检测右边是否有障碍物
def RightObstacleDetected(psValues):
    if psValues[5] > 500.0 or psValues[6] > 500.0 or psValues[7] > 500.0:
        return True
    else:
        return False

def changeDirection(robot, psValues, leftspeed, rightspeed):
    # 左边有障碍，右转
    if LeftObstacleDetected(psValues):
        leftspeed = 0.7 * MAX_SPEED
        rightspeed = -0.7 * MAX_SPEED
    elif RightObstacleDetected(psValues):
        leftspeed = -0.7 * MAX_SPEED
        rightspeed = 0.7 * MAX_SPEED         
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:

    psValues = [ds.getValue() for ds in ds_list]
    
    leftspeed = 0.7 * MAX_SPEED
    rightspeed = 0.7 * MAX_SPEED
    # changeDirection(robot, psValues, leftspeed, rightspeed)  # 检验是否需要转向

    if LeftObstacleDetected(psValues):
        leftspeed = -1 * MAX_SPEED
        rightspeed = 1 * MAX_SPEED
    elif RightObstacleDetected(psValues):
        leftspeed = 1 * MAX_SPEED
        rightspeed = -1 * MAX_SPEED 
        
    leftMotor.setVelocity(leftspeed)
    rightMotor.setVelocity(rightspeed)
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
