from controller import Motor, PositionSensor, TouchSensor, InertialUnit, Camera, Lidar, Supervisor
import numpy as np
import sys
import math
import time
OUTPUT_LOG = True

TIME_STEP = 16

MAX_WHEEL_SPEED = 3.0       # Maximum velocity for the wheels [rad / s]
WHEELS_DISTANCE = 0.4492    # Distance between 2 caster wheels (the four wheels are located in square) [m]
SUB_WHEELS_DISTANCE = 0.098 # Distance between 2 sub wheels of a caster wheel [m]
WHEEL_RADIUS = 0.08         # Wheel radius
TOLERANCE_1 = 0.006
TOLERANCE_2 = 0.0015

def ALM_EQUAL_1(a, b) -> bool:
    return (a < b + TOLERANCE_1) and (a > b - TOLERANCE_1)
    
def ALM_EQUAL_2(a, b) -> bool:
    return (a < b + TOLERANCE_2) and (a > b - TOLERANCE_2)
    
def LOG(text: str, enable: bool=OUTPUT_LOG) -> None:
    print(text)


class Env_Info():
    def __init__(self, 
                 bread: int,
                 lettuce: int,
                 beef: int,
                 cheese: int,
                 tomato: int,
                 rb_pos_bread: np.array, 
                 rb_pos_lettuce: np.array,
                 rb_pos_beef: np.array, 
                 rb_pos_cheese: np.array,
                 rb_pos_tomato: np.array,
                 rb_pos_hamburger: np.array, 
                 offset_hor: float,
                 offset_ver: float) -> None:
                 
        # Infomation about the env
        self.ingredients = {'bread': bread,
                            'lettuce': lettuce,
                            'beef': beef,
                            'cheese': cheese,
                            'tomato': tomato,
                            'hamburger': 0}
        
        # Position
        self.rb_pos = {'bread': rb_pos_bread,
                       'lettuce': rb_pos_lettuce,
                       'beef': rb_pos_beef,
                       'cheese': rb_pos_cheese,
                       'tomato': rb_pos_tomato,
                       'hamburger': rb_pos_hamburger}
        self.rb_height = 0.78
        
        # Offset
        self.offset_hor = offset_hor
        self.offset_ver = offset_ver

    def get_rb_target_pos(self, target: str, hand: str) -> np.array:

        if hand == 'left':
            offset = [0, -self.offset_hor, self.ingredients[target] * self.offset_ver]
            return self.rb_pos[target] + np.array(offset)
        elif hand == 'right':
            offset = [0, self.offset_hor, self.ingredients[target] * self.offset_ver]
            return self.rb_pos[target] + np.array(offset)

    def update_ingredients(self, target_obj: str) -> None:
        if target_obj == 'hamburger':
            self.ingredients[target_obj] += 1
        else:
            self.ingredients[target_obj] -= 1   

class PR2():
    def __init__(self, supervisor: Supervisor, environment: Env_Info) -> None:

        self.supervisor = supervisor
        self.env = environment

        # PR2 motors' device
        self.wheel_motors = []
        self.rotation_motors = []
        self.left_arm_motors = []
        self.right_arm_motors = []
        self.right_finger_motors = []
        self.left_finger_motors = []
        self.head_tilt_motor = []
        self.torso_motor = []

        # PR2 sensors' device
        self.wheel_sensors = []
        self.rotation_sensors = []
        self.left_arm_sensors = []
        self.right_arm_sensors = []
        self.right_finger_sensors = []
        self.left_finger_sensors = []

        self.torso_sensor = []
        self.imu_sensor = []
        self.left_finger_contact_sensors = []
        self.right_finger_contact_sensors = []
        self.wide_stereo_l_stereo_camera_sensor = []
        self.wide_stereo_r_stereo_camera_sensor = []
        self.high_def_sensor = []
        self.r_forearm_cam_sensor = []
        self.l_forearm_cam_sensor = []
        self.laser_tilt = []
        self.base_laser = []

        # PR2 motors' name
        wheel_motors_name = ['fl_caster_l_wheel_joint',
                             'fl_caster_r_wheel_joint',
                             'fr_caster_l_wheel_joint',
                             'fr_caster_r_wheel_joint',
                             'bl_caster_l_wheel_joint',
                             'bl_caster_r_wheel_joint',
                             'br_caster_l_wheel_joint',
                             'br_caster_r_wheel_joint']
        rotation_motors_name = ['fl_caster_rotation_joint',
                                'fr_caster_rotation_joint',
                                'bl_caster_rotation_joint',
                                'br_caster_rotation_joint']
        left_arm_motors_name = ['l_shoulder_pan_joint',
                                'l_shoulder_lift_joint',
                                'l_upper_arm_roll_joint',
                                'l_elbow_flex_joint',
                                'l_wrist_roll_joint']
        right_arm_motors_name = ['r_shoulder_pan_joint',
                                 'r_shoulder_lift_joint',
                                 'r_upper_arm_roll_joint',
                                 'r_elbow_flex_joint',
                                 'r_wrist_roll_joint']
        left_finger_motors_name = ['l_gripper_l_finger_joint',
                                   'l_gripper_r_finger_joint',
                                   'l_gripper_l_finger_tip_joint',
                                   'l_gripper_r_finger_tip_joint']
        right_finger_motors_name = ['r_gripper_l_finger_joint',
                                    'r_gripper_r_finger_joint',
                                    'r_gripper_l_finger_tip_joint',
                                    'r_gripper_r_finger_tip_joint']
        head_tilt_motor_name = ['head_tilt_joint']
        torso_motor_name = ['torso_lift_joint']

        # PR2 sensors' name
        torso_sensor_name = ['torso_lift_joint_sensor']
        imu_sensor_name = ['imu_sensor']
        left_finger_contact_sensors_name = ['l_gripper_l_finger_tip_contact_sensor',
                                            'l_gripper_r_finger_tip_contact_sensor']
        right_finger_contact_sensors_name = ['r_gripper_l_finger_tip_contact_sensor',
                                             'r_gripper_r_finger_tip_contact_sensor']
        wide_stereo_l_stereo_camera_sensor_name = ['wide_stereo_l_stereo_camera_sensor']
        wide_stereo_r_stereo_camera_sensor_name = ['wide_stereo_r_stereo_camera_sensor']
        high_def_sensor_name = ['high_def_sensor']
        r_forearm_cam_sensor_name = ['r_forearm_cam_sensor']
        l_forearm_cam_sensor_name = ['l_forearm_cam_sensor']
        laser_tilt_name = ['laser_tilt']
        base_laser_name = ['base_laser']
        
        # Get robot node
        self.robot_node = None
        self._set_robot_node()
        
        # Retrieve all the PR2 devices
        self._initialize_devices(wheel_motors_name, rotation_motors_name,
                                 left_arm_motors_name, right_arm_motors_name,
                                 left_finger_motors_name,
                                 right_finger_motors_name,
                                 head_tilt_motor_name, torso_motor_name,
                                 torso_sensor_name, imu_sensor_name,
                                 left_finger_contact_sensors_name,
                                 right_finger_contact_sensors_name,
                                 wide_stereo_l_stereo_camera_sensor_name,
                                 wide_stereo_r_stereo_camera_sensor_name,
                                 high_def_sensor_name,
                                 r_forearm_cam_sensor_name,
                                 l_forearm_cam_sensor_name,
                                 laser_tilt_name, base_laser_name)

        # Enable devices
        self._enable_devices()

        # Set initial position
        self.set_initial_arm_position()
        
    def _set_robot_node(self) -> None:
        """Set robot node"""
        
        self.robot_node = self.supervisor.getFromDef('pr2_robot')
        
    def get_orientation(self) -> float:
    
        rot = self.robot_node.getOrientation()
        matrix = np.array([[rot[0], rot[1], rot[2]],
                           [rot[3], rot[4], rot[5]],
                           [rot[6], rot[7], rot[8]]])
        true_ori = np.round(matrix @ np.array([[1], [0], [0]]), decimals=3)
        curr_rad = np.arctan2(true_ori[1], true_ori[0])

        if curr_rad > np.pi / 2:
            return 2 * np.pi - curr_rad
        elif curr_rad < -np.pi / 2:
            return -curr_rad
        else:
            return np.pi - curr_rad

    def get_position(self) -> np.array:
    
        pos = self.robot_node.getPosition()
        return np.round(pos, decimals=3)
    
    def _initialize_devices(self, wheel_motors_name: list,
                            rotation_motors_name: list,
                            left_arm_motors_name: list,
                            right_arm_motors_name: list,
                            left_finger_motors_name: list,
                            right_finger_motors_name: list,
                            head_tilt_motor_name: list,
                            torso_motor_name: list,
                            torso_sensor_name: list,
                            imu_sensor_name: list,
                            left_finger_contact_sensors_name: list,
                            right_finger_contact_sensors_name: list,
                            wide_stereo_l_stereo_camera_sensor_name: list,
                            wide_stereo_r_stereo_camera_sensor_name: list,
                            high_def_sensor_name: list,
                            r_forearm_cam_sensor_name: list,
                            l_forearm_cam_sensor_name: list,
                            laser_tilt_name: list,
                            base_laser_name: list) -> None:
        """ Retrieve all the PR2 devices"""

        for name in wheel_motors_name:
            motor = Motor(name)
            self.wheel_motors.append(motor)
            sensor = motor.getPositionSensor()
            self.wheel_sensors.append(sensor)
        for name in rotation_motors_name:
            motor = Motor(name)
            self.rotation_motors.append(motor)
            sensor = motor.getPositionSensor()
            self.rotation_sensors.append(sensor)
        for name in left_arm_motors_name:
            motor = Motor(name)
            self.left_arm_motors.append(motor)
            sensor = motor.getPositionSensor()
            self.left_arm_sensors.append(sensor)
        for name in right_arm_motors_name:
            motor = Motor(name)
            self.right_arm_motors.append(motor)
            sensor = motor.getPositionSensor()
            self.right_arm_sensors.append(sensor)
        for name in left_finger_motors_name:
            motor = Motor(name)
            self.left_finger_motors.append(motor)
            sensor = motor.getPositionSensor()
            self.left_finger_sensors.append(sensor)
        for name in right_finger_motors_name:
            motor = Motor(name)
            self.right_finger_motors.append(motor)
            sensor = motor.getPositionSensor()
            self.right_finger_sensors.append(sensor)

        for name in head_tilt_motor_name:
            self.head_tilt_motor.append(Motor(name))
        for name in torso_motor_name:
            self.torso_motor.append(Motor(name))
        
        for name in torso_sensor_name:
            self.torso_sensor.append(PositionSensor(name))

        for name in imu_sensor_name:
            self.imu_sensor.append(InertialUnit(name))
        
        for name in left_finger_contact_sensors_name:
            self.left_finger_contact_sensors.append(TouchSensor(name))
        for name in right_finger_contact_sensors_name:
            self.right_finger_contact_sensors.append(TouchSensor(name))

        for name in wide_stereo_l_stereo_camera_sensor_name:
            self.wide_stereo_l_stereo_camera_sensor.append(Camera(name))
        for name in wide_stereo_r_stereo_camera_sensor_name:
            self.wide_stereo_r_stereo_camera_sensor.append(Camera(name))
        for name in high_def_sensor_name:
            self.high_def_sensor.append(Camera(name))
        for name in r_forearm_cam_sensor_name:
            self.r_forearm_cam_sensor.append(Camera(name))
        for name in l_forearm_cam_sensor_name:
            self.l_forearm_cam_sensor.append(Camera(name))

        for name in laser_tilt_name:
            self.laser_tilt.append(Lidar(name))
        for name in base_laser_name:
            self.base_laser.append(Lidar(name))

    def _enable_devices(self) -> None:
        """Enable the robot devices"""

        for i in range(len(self.wheel_motors)):
            self.wheel_sensors[i].enable(TIME_STEP)
            self.wheel_motors[i].setPosition(float('+inf'))
            self.wheel_motors[i].setVelocity(0.0)

        for sensor in self.rotation_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.left_finger_contact_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.right_finger_contact_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.left_finger_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.right_finger_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.left_arm_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.right_arm_sensors:
            sensor.enable(TIME_STEP)
        for sensor in self.torso_sensor:
            sensor.enable(TIME_STEP)

        # Other devices are not used in this simulation

    def set_wheels_speed(self, speed: float) -> None:
        """Set the speeds of the robot wheels"""

        for motor in self.wheel_motors:
            motor.setVelocity(speed)

    def enable_passive_wheels(self, enable: bool, torques: np.array) -> np.array:
        """Enable/disable the torques on the wheels motors"""

        if enable:
            for i, motor in enumerate(self.wheel_motors):
                torques[i] = motor.getAvailableTorque()
                motor.setAvailableTorque(0)
        else:
            for i, motor in enumerate(self.wheel_motors):
                motor.setAvailableTorque(torques[i])

        return torques

    def set_rotation_wheels_angles(self, fl: float, fr: float, bl: float, 
                                   br: float, wait_on_feedback: bool) -> None:
        """
        Set the rotation wheels angles.
        If wait_on_feedback is true, the function is left when the rotational 
        motors have reached their target positions.
        """
        pos = [fl, fr, bl, br]

        if wait_on_feedback:
            self.set_wheels_speed(0)
            torques = self.enable_passive_wheels(True, np.zeros(8))

        for i, motor in enumerate(self.rotation_motors):
            motor.setPosition(pos[i])

        if wait_on_feedback:
            while True:
                all_reached = True
                for i, sensor in enumerate(self.rotation_sensors):
                    current_position = sensor.getValue()
                    if not ALM_EQUAL_2(current_position, pos[i]):
                        all_reached = False
                        break

                if all_reached:
                    break
                elif self.supervisor.step(TIME_STEP) == -1:
                    break

            self.enable_passive_wheels(False, torques)

    def robot_rotate(self, angle: float) -> None:
        """
        High level function to rotate the robot around itself of a given angle [rad]
        Note: the angle can be negative
        """

        self.set_wheels_speed(0)
        self.set_rotation_wheels_angles(3 * np.pi / 4.0, np.pi / 4.0, 
                                        -3 * np.pi / 4.0, -np.pi / 4.0, True)
        max_wheel_speed = MAX_WHEEL_SPEED if angle > 0 else -MAX_WHEEL_SPEED
        self.set_wheels_speed(max_wheel_speed)

        initial_wheel0_position = self.wheel_sensors[0].getValue()
        # Expected travel distance done by the wheel
        expected_travel_distance = abs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE))

        while True:
            wheel0_position = self.wheel_sensors[0].getValue()
            # Travel distance done by the wheel
            wheel0_travel_distance = abs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))

            if ALM_EQUAL_1(wheel0_travel_distance, expected_travel_distance):
                break

            # Reduce the speed before reaching the target
            if expected_travel_distance - wheel0_travel_distance < 0.025:
                self.set_wheels_speed(0.025 * max_wheel_speed)

            if self.supervisor.step(TIME_STEP) == -1:
                break

        # Reset wheels
        self.set_rotation_wheels_angles(0, 0, 0, 0, True)
        self.set_wheels_speed(0)

    def robot_go_forward(self, distance: float) -> None:
        """
        High level function to go forward for a given distance [m]
        Note: the distance can be negative
        """
        
        max_wheel_speed = MAX_WHEEL_SPEED if distance > 0 else -MAX_WHEEL_SPEED
        self.set_wheels_speed(max_wheel_speed)

        initial_wheel0_position = self.wheel_sensors[0].getValue()

        while True:
            wheel0_position = self.wheel_sensors[0].getValue()
            # Travel distance done by the wheel
            wheel0_travel_distance = abs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position))

            if ALM_EQUAL_2(wheel0_travel_distance, abs(distance)):
                break

            # Reduce the speed before reaching the target
            if abs(distance) - wheel0_travel_distance < 0.025:
                self.set_wheels_speed(0.01 * max_wheel_speed)

            if self.supervisor.step(TIME_STEP) == -1:
                break

        self.set_wheels_speed(0.0)

    def set_gripper(self, gripper_inx: int, open: bool, torque_when_gripping: float,
                    wait_on_feedback: bool,target_open_value:float) -> None:
        """
        Open or close the gripper.
        If wait_on_feedback is True, the gripper is stopped either when the 
        target is reached, or either when something has been gripped
        """
        
        motors = []
        motors.append(self.left_finger_motors[0] if gripper_inx == 0 else self.right_finger_motors[0])
        motors.append(self.left_finger_motors[1] if gripper_inx == 0 else self.right_finger_motors[1])
        motors.append(self.left_finger_motors[2] if gripper_inx == 0 else self.right_finger_motors[2])
        motors.append(self.left_finger_motors[3] if gripper_inx == 0 else self.right_finger_motors[3])

        sensors = []
        sensors.append(self.left_finger_sensors[0] if gripper_inx == 0 else self.right_finger_sensors[0])
        sensors.append(self.left_finger_sensors[1] if gripper_inx == 0 else self.right_finger_sensors[1])
        sensors.append(self.left_finger_sensors[2] if gripper_inx == 0 else self.right_finger_sensors[2])
        sensors.append(self.left_finger_sensors[3] if gripper_inx == 0 else self.right_finger_sensors[3])

        contacts = []
        contacts.append(self.left_finger_contact_sensors[0] if gripper_inx == 0 else self.right_finger_contact_sensors[0])
        contacts.append(self.left_finger_contact_sensors[1] if gripper_inx == 0 else self.right_finger_contact_sensors[1])

        first_call = True
        max_torque = 0
        if first_call:
            max_torque = motors[0].getAvailableTorque()
            first_call = False

        for motor in motors:
            motor.setAvailableTorque(max_torque)

        if open:
            # target_open_value = 0.2
            for motor in motors:
                motor.setPosition(target_open_value)

            if wait_on_feedback:
                while not ALM_EQUAL_1(sensors[0].getValue(), target_open_value):
                    if self.supervisor.step(TIME_STEP) == -1:
                        break

        else:
            target_close_value = 0
            for motor in motors:
                motor.setPosition(target_close_value)

            if wait_on_feedback:
                # Wait until the 2 touch sensors are fired or the target value is reached
                while (
                    (contacts[0].getValue() == 0 or contacts[1].getValue() == 0) 
                    and 
                    not ALM_EQUAL_2(sensors[0].getValue(), target_close_value)
                ):
                    if self.supervisor.step(TIME_STEP) == -1:
                        break

                current_position = sensors[0].getValue()
                for motor in motors:
                    motor.setAvailableTorque(torque_when_gripping)
                    motor.setPosition(max(0, 0.95 * current_position))

    def set_arm_position(self, shoulder_roll: float, shoulder_lift: float,
                         upper_arm_roll: float, elbow_lift: float,
                         wrist_roll: float, wait_on_feedback: bool,
                         arm_idx: int) -> None:
        """
        Set the right arm position (forward kinematics)
        If wait_on_feedback is enabled, the function is left when the target is reached.
        """

        if arm_idx == 0:
            arm_motors = self.left_arm_motors
            arm_sensors = self.left_arm_sensors
        else:
            arm_motors = self.right_arm_motors
            arm_sensors = self.right_arm_sensors

        arm_motors[0].setPosition(shoulder_roll)
        arm_motors[1].setPosition(shoulder_lift)
        arm_motors[2].setPosition(upper_arm_roll)
        arm_motors[3].setPosition(elbow_lift)
        arm_motors[4].setPosition(wrist_roll)

        if wait_on_feedback:
            while (
                not ALM_EQUAL_1(arm_sensors[0].getValue(), shoulder_roll) or
                not ALM_EQUAL_1(arm_sensors[1].getValue(), shoulder_lift) or
                not ALM_EQUAL_1(arm_sensors[2].getValue(), upper_arm_roll) or
                not ALM_EQUAL_1(arm_sensors[3].getValue(), elbow_lift) or
                not ALM_EQUAL_1(arm_sensors[4].getValue(), wrist_roll)
                ):
                if self.supervisor.step(TIME_STEP) == -1:
                    break

    def set_torso_height(self, height: float, wait_on_feedback: bool) -> None:
        """
        Set the torso height
        If wait_on_feedback is enabled, the function is left when the target is reached.
        """

        print('env.rb_height:', self.env.rb_height)
        print('height:', height)

        h_diff = height - self.env.rb_height 
        if h_diff < 0:          
            h_diff = height - 0.78
        elif h_diff >= 0 and h_diff < 0.01:
            h_diff = self.torso_sensor[0].getValue()  

        # h_diff = np.clip(h_diff, 0, 0.33)
        self.torso_motor[0].setPosition(h_diff)
        
        print('h_diff:', h_diff)
        if wait_on_feedback:
            while not ALM_EQUAL_2(self.torso_sensor[0].getValue(), h_diff):
                if self.supervisor.step(TIME_STEP) == -1:
                    break
        self.env.rb_height = self.env.rb_height + self.torso_sensor[0].getValue()         

    def set_initial_arm_position(self) -> None:
        """Convenient initial position"""

        # Go to the initial position
        self.set_arm_position(0, 0, 0, 0, 1.57, False, 0)
        self.set_arm_position(0, 0, 0, 0, 1.57, False, 1)
        
        self.set_gripper(0, True, 0, False, 0.2)
        self.set_gripper(1, True, 0, False, 0.2)
        
    def move_to_object_location(self, target_pos: np.array) -> None:
        """
        move the robot to a appropriate postion from initial position so that 
        the food can be successfully reached by grab_object function, note that the initial location 
        is (0.25, 0, 0)
        """

        self.set_torso_height(target_pos[2], True)
        self.set_wheels_speed(0)
        self.set_rotation_wheels_angles(np.pi / 2.0, np.pi / 2.0, 
                                        np.pi / 2.0, np.pi / 2.0, True)
        curr_pos = self.get_position()                                    
        self.robot_go_forward(target_pos[1] - curr_pos[1])
        
        self.set_wheels_speed(0)
        self.set_rotation_wheels_angles(0, 0, 0, 0, True) 
        
        curr_pos = self.get_position()
        self.robot_go_forward(target_pos[0] - curr_pos[0])

    def grab_object(self, object_hand: str) -> None:
        """
        grab the objects from the table A
        """
        if object_hand == "left":
            arm = 0
            gripper = 0
        elif object_hand == "right":
            arm = 1
            gripper = 1
        else:
            print ("Error: Wrong string entered")

        self.set_gripper(gripper, False, 10.0, True, 0.15)
        self.set_arm_position(0.0, -0.06, 0.0, 0, 1.57, True, arm)
          
    def move_to_initial_position(self,
                                 go_to_take: bool,
                                 init_pos: np.array=np.zeros(3)) -> None:
        """
        move the robot with objects in hands from loc to another table B
        """
        curr_pos = self.get_position()
        self.set_wheels_speed(0)
        self.set_rotation_wheels_angles(0, 0, 0, 0, True)
        if go_to_take:
            self.robot_go_forward(init_pos[0] - curr_pos[0])

        else:
            self.robot_go_forward(init_pos[0] + curr_pos[0])
        
        self.set_wheels_speed(0)
        self.set_rotation_wheels_angles(np.pi / 2.0, np.pi / 2.0, 
                                        np.pi / 2.0, np.pi / 2.0, True)
        
        if go_to_take:
            self.robot_go_forward(init_pos[1] - curr_pos[1])

        else:
            self.robot_go_forward(init_pos[1] + curr_pos[1])

        
    def put_down_object(self, target_hand: str)-> None:
        """
        put down the objects grabing by hands
        """
        if target_hand == "left":
            arm = 0
            gripper = 0
              
        elif target_hand == "right":
            arm = 1
            gripper = 1

        self.set_arm_position(0, 0, 0, 0, 1.57, True, arm)
        #self.set_arm_position(0.0, 0.55, 0.0, -0.52, 1.57, False, arm)
        self.set_gripper(gripper, True, 0, True, 0.2)

    def take(self, target_hand: str, target_obj: str) -> None:
        assert target_obj in list(self.env.ingredients.keys()), "Unknown target object"
        assert target_hand in ['left', 'right'], "Unknown target hand"

        LOG("take -> get_rb_target_pos")
        target_pos = self.env.get_rb_target_pos(target_obj, target_hand)
        LOG("take -> move_to_object_location")
        self.move_to_object_location(target_pos)
        LOG("take -> grab_object")
        self.grab_object(target_hand)
        LOG("take -> move_to_initial_position")
        self.move_to_initial_position(go_to_take=True)
        LOG("put down arm")
        LOG("take -> update_ingredients")
        self.env.update_ingredients(target_obj)

    def put(self, target_hand: str, target_obj: str="hamburger") -> None:
        assert target_obj == "hamburger", "Can only put hamburger"
        assert target_hand in ['left', 'right'], "Unknown target hand"
   
        LOG("take -> DONE")
        target_pos = self.env.get_rb_target_pos(target_obj, target_hand)
        LOG("put -> move_to_object_location")
        self.move_to_object_location(target_pos)
        LOG("put -> put_down_object")
        self.put_down_object(target_hand)
        LOG("put -> move_to_initial_position")
        self.move_to_initial_position(go_to_take=False)
        LOG("put -> update_ingredients")
        self.env.update_ingredients(target_obj)
        LOG("put -> DONE")
        # self.set_initial_arm_position()
        # LOG("arm initial")
        
    def rotate_180(self) -> None:
        
        LOG("rotate_180 -> get_orientation")
        target_ang = self.get_orientation()
        LOG("rotate_180 -> robot_rotate")
        self.robot_rotate(target_ang)
        LOG("rotate_180 -> DONE")

if __name__ == "__main__":
    robot = Supervisor()
    env = Env_Info(bread=6,
                   lettuce=6,
                   beef=6,
                   cheese=6,
                   tomato=6,
                   rb_pos_bread=np.array([0.4, 0.78, 0.76]), 
                   rb_pos_lettuce=np.array([0.4, 0.42, 0.76]),
                   rb_pos_beef=np.array([0.4, 0.06, 0.76]),
                   rb_pos_cheese=np.array([0.4, -0.3, 0.76]),
                   rb_pos_tomato=np.array([0.4, -0.66, 0.76]),
                   rb_pos_hamburger=np.array([0.4, -0.1, 0.84]),
                   offset_hor=0.20,
                   offset_ver=0.04)
    pr2 = PR2(robot, env)
    pr2.set_initial_arm_position()
    
    # Loop starts

    from initial_prompt_for_chatGPT import env_example, intitial_prompts
    from GUI import GUI

    def take(target_hand: str, target_obj: str):
        pr2.take(target_hand, target_obj)
        
    def put(target_hand: str, target_obj: str="hamburger"):
        pr2.put(target_hand, target_obj)
        
    def rotate_180():
        pr2.rotate_180()
        
    def execute_order_66(env):
        prompts,instructions = intitial_prompts(env)
        chatGPT_response = GUI(prompts,instructions)

        for cur_task in chatGPT_response["task_cohesion"]["task_sequence"]:
            exec(cur_task)

    def begin():
        env = env_example()
        execute_order_66(env)

    begin()

    """pr2.take(target_hand='right', target_obj= 'bread')
    pr2.take(target_hand='left', target_obj='cheese')
    pr2.rotate_180()    
    pr2.put(target_hand='right')
    pr2.put(target_hand='left')
    pr2.rotate_180()
    pr2.move_to_initial_position(go_to_take = True)

    pr2.take(target_hand='right', target_obj='beef')
    pr2.take(target_hand='left', target_obj='tomato')
    pr2.rotate_180()
    pr2.put(target_hand='right')
    pr2.put(target_hand='left')    
    pr2.rotate_180()
    pr2.move_to_initial_position(go_to_take = True)
    
    pr2.take(target_hand='right', target_obj='bread')
    pr2.rotate_180()
    pr2.put(target_hand='right')
    pr2.rotate_180()
    pr2.move_to_initial_position(go_to_take = True)"""
