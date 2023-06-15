from controller import Keyboard, Motor, Robot
import numpy as np

MAX_SPEED = 7.0
N_PARTS = 12

def check_keyboard(keyboard, motor_left, motor_right):
    speeds_left = 0.0
    speeds_right = 0.0

    key = keyboard.getKey()
    if (key >= 0):
        if key == keyboard.UP:
            speeds_left = MAX_SPEED
            speeds_right = MAX_SPEED

        elif key == keyboard.DOWN:
            speeds_left = -MAX_SPEED
            speeds_right = -MAX_SPEED

        elif key == keyboard.RIGHT:
            speeds_left = MAX_SPEED
            speeds_right = -MAX_SPEED

        elif key == keyboard.LEFT:
            speeds_left = -MAX_SPEED
            speeds_right = MAX_SPEED

    Motor(motor_left).setVelocity(speeds_left)
    Motor(motor_right).setVelocity(speeds_right)

robot = Robot()
time_step = int(robot.getBasicTimeStep())

names = ["head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
         "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
         "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint"]
target_pos = [0.24, -0.67, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, float('+inf'), float('+inf')]

robot_parts = []
for i in range(N_PARTS):
    #robot_parts.append(robot.getDevice(names[i]))
    robot_parts = names

    Motor(robot_parts[i]).setVelocity(Motor(robot_parts[i]).getMaxVelocity() / 2.0)
    Motor(robot_parts[i]).setPosition(target_pos[i])

print("You can drive this robot by selecting the 3D window and pressing the keyboard arrows.")
keyboard = Keyboard()
keyboard.enable(time_step)
initialTime = robot.getTime()

while robot.step(time_step) != -1:
    check_keyboard(keyboard, robot_parts[10], robot_parts[11])

    time = robot.getTime() - initialTime
    Motor(robot_parts[8]).setPosition(0.3 * np.sin(5.0 * time) - 0.3)
