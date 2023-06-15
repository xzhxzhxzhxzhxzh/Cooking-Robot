from controller import Motor, PositionSensor, TouchSensor, InertialUnit, Robot, Camera, Lidar
import numpy as np
import sys

TIME_STEP = 16

MAX_WHEEL_SPEED = 3.0       # Maximum velocity for the wheels [rad / s]
WHEELS_DISTANCE = 0.4492    # Distance between 2 caster wheels (the four wheels are located in square) [m]
SUB_WHEELS_DISTANCE = 0.098 # Distance between 2 sub wheels of a caster wheel [m]
WHEEL_RADIUS = 0.08         # Wheel radius

TOLERANCE = 0.05 # arbitrary value

def almost_equal(a, b) -> bool:
    return (a < b + TOLERANCE) and (a > b - TOLERANCE)

class PR2(Robot):
    def __init__(self) -> None:
        super().__init__()

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
        self.set_initial_position()

        # Go to the initial position
        self.set_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, True, 0)
        self.set_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, True, 1)
        self.robot_go_forward(0.35)

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
                    if not almost_equal(current_position, pos[i]):
                        all_reached = False
                        break

                if all_reached:
                    break
                elif self.step(TIME_STEP) == -1:
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

            if wheel0_travel_distance > expected_travel_distance:
                break

            # Reduce the speed before reaching the target
            if expected_travel_distance - wheel0_travel_distance < 0.025:
                self.set_wheels_speed(0.1 * max_wheel_speed)

            if self.step(TIME_STEP) == -1:
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

            if wheel0_travel_distance > abs(distance):
                break

            # Reduce the speed before reaching the target
            if abs(distance) - wheel0_travel_distance < 0.025:
                self.set_wheels_speed(0.1 * max_wheel_speed)

            if self.step(TIME_STEP) == -1:
                break

        self.set_wheels_speed(0.0)

    def set_gripper(self, left: bool, open: bool, torque_when_gripping: float,
                    wait_on_feedback: bool) -> None:
        """
        Open or close the gripper.
        If wait_on_feedback is True, the gripper is stopped either when the 
        target is reached, or either when something has been gripped
        """
        
        motors = []
        motors.append(self.left_finger_motors[0] if left else self.right_finger_motors[0])
        motors.append(self.left_finger_motors[1] if left else self.right_finger_motors[1])
        motors.append(self.left_finger_motors[2] if left else self.right_finger_motors[2])
        motors.append(self.left_finger_motors[3] if left else self.right_finger_motors[3])

        sensors = []
        sensors.append(self.left_finger_sensors[0] if left else self.right_finger_sensors[0])
        sensors.append(self.left_finger_sensors[1] if left else self.right_finger_sensors[1])
        sensors.append(self.left_finger_sensors[2] if left else self.right_finger_sensors[2])
        sensors.append(self.left_finger_sensors[3] if left else self.right_finger_sensors[3])

        contacts = []
        contacts.append(self.left_finger_contact_sensors[0] if left else self.right_finger_contact_sensors[0])
        contacts.append(self.left_finger_contact_sensors[1] if left else self.right_finger_contact_sensors[1])

        first_call = True
        max_torque = 0
        if first_call:
            max_torque = motors[0].getAvailableTorque()
            first_call = False

        for motor in motors:
            motor.setAvailableTorque(max_torque)

        if open:
            target_open_value = 0.5
            for motor in motors:
                motor.setPosition(target_open_value)

            if wait_on_feedback:
                while not almost_equal(sensors[0].getValue(), target_open_value):
                    if self.step(TIME_STEP) == -1:
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
                    not almost_equal(sensors[0].getValue(), target_close_value)
                ):
                    if self.step(TIME_STEP) == -1:
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
                not almost_equal(arm_sensors[0].getValue(), shoulder_roll) or
                not almost_equal(arm_sensors[1].getValue(), shoulder_lift) or
                not almost_equal(arm_sensors[2].getValue(), upper_arm_roll) or
                not almost_equal(arm_sensors[3].getValue(), elbow_lift) or
                not almost_equal(arm_sensors[4].getValue(), wrist_roll)
                ):
                if self.step(TIME_STEP) == -1:
                    break

    def set_torso_height(self, height: float, wait_on_feedback: bool) -> None:
        """
        Set the torso height
        If wait_on_feedback is enabled, the function is left when the target is reached.
        """

        self.torso_motor[0].setPosition(height)
        if wait_on_feedback:
            while not almost_equal(self.torso_sensor[0].getValue(), height):
                if self.step(TIME_STEP) == -1:
                    break

    def set_initial_position(self) -> None:
        """Convenient initial position"""
        
        self.set_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, False, 0)
        self.set_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, False, 1)

        self.set_gripper(False, True, 0, False)
        self.set_gripper(True, True, 0, False)

        self.set_torso_height(0.2, True)


pr2 = PR2()

# Main loop
while True:
    # Close the gripper with forcefeedback
    pr2.set_gripper(True, False, 20.0, True)
    pr2.set_gripper(False, False, 20.0, True)
    # Lift the arms
    pr2.set_arm_position(0.0, 0.5, 0.0, -1.0, 0.0, True, 0)
    pr2.set_arm_position(0.0, 0.5, 0.0, -1.0, 0.0, True, 1)
    # Go to the other table
    pr2.robot_go_forward(-0.35)
    pr2.robot_rotate(np.pi)
    pr2.robot_go_forward(0.35)
    # Move the arms down
    pr2.set_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, True, 0)
    pr2.set_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, True, 1)
    # Open the grippers
    pr2.set_gripper(True, True, 0.0, True)
    pr2.set_gripper(False, True, 0.0, True)
