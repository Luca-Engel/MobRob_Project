import numpy as np
import math
from filterpy.kalman import KalmanFilter


class ThymioKalmanFilter:
    def __init__(self, position_thymio_camera_est, direction_thymio_camera_est):
        self.kf = KalmanFilter(dim_x=4, dim_z=4)

        dir_x, dir_y = direction_thymio_camera_est
        angle = math.atan2(dir_y, dir_x)
        if angle < 0:
            angle += 2 * math.pi

        print("angle", angle)
        # Initial state (x, y, angle)
        self.kf.x = np.array([position_thymio_camera_est[0], position_thymio_camera_est[1], angle, 1])

        print("initial x", self.kf.x)

        # State transition matrix
        # first line is for x, second for y, third for angle
        self.kf.F = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        # Measurement function
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        # Measurement noise covariance
        self.kf.R = np.array([[0.1, 0, 0, 0],
                              [0, 0.1, 0, 0],
                              [0, 0, 0.1, 0],
                              [0, 0, 0, 0.1]])

        # Process noise covariance
        self.kf.Q = np.array([[0.001, 0, 0, 0],
                              [0, 0.001, 0, 0],
                              [0, 0, 0.001, 0],
                              [0, 0, 0, 0.001]])

        # Control input matrix should account for the position and orientation of the robot
        self.kf.B = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]]) # TODO: check if it needs to be a 3x1 matrix

    def update(self, position_camera_est, direction_camera_est, left_wheel_speed, right_wheel_speed):
        print("-------------------")

        scaled_left_wheel_speed = left_wheel_speed * 0.0002
        scaled_right_wheel_speed = right_wheel_speed * 0.0002

        # Update the state transition matrix based on wheel speeds
        dt = 0.2  # Time step (you may need to adjust this based on your system)
        v = (scaled_left_wheel_speed + scaled_right_wheel_speed) / 2.0
        w = (right_wheel_speed - left_wheel_speed) / 0.2  # Assuming wheelbase of 0.2 (you may need to adjust this)



        min_period = 9  # seconds for 100 and -100 speed
        right_wheel_speed = max(min(right_wheel_speed, 100), -100)
        left_wheel_speed = max(min(left_wheel_speed, 100), -100)

        # period = (right_wheel_speed - left_wheel_speed) / 200.0 * min_period  # positive sense is counterclockwise
        period = 0
        if abs(right_wheel_speed - left_wheel_speed) > 0.5: # avoid division by 0 and large values
            period = (200 / (right_wheel_speed - left_wheel_speed)) * min_period  # positive sense is counterclockwise
        w = 0
        if period != 0:
            w = 2 * np.pi / period



        if abs(v) < 20:
            # Pure rotation without translation
            self.kf.F[0, 3] = 1 # -w * math.sin(self.kf.x[2]) * dt
            self.kf.F[1, 3] = 1 # w * math.cos(self.kf.x[2]) * dt
        else:
            # Translation with rotation
            self.kf.F[0, 3] = v * math.cos(self.kf.x[2]) * 0.25 # dt   # becomes x_k+1 = x_k + v * cos(theta) * dt
            self.kf.F[1, 3] = v * math.sin(self.kf.x[2]) * 0.25 # dt   # becomes y_k+1 = y_k + v * sin(theta) * dt
            self.kf.F[2, 3] = w * dt # becomes theta_k+1 = theta_k + w * dt





        self.kf.predict()

        # Update the direction estimation based on camera estimation and wheel speed difference
        direction_angle_camera = math.atan2(direction_camera_est[1], direction_camera_est[0])  # - self.kf.x[2]
        direction_angle_camera = direction_angle_camera % (2 * math.pi)


        # angle_diff_wheels = w * dt
        # var_angle_diff = direction_angle_camera - self.kf.x[2]

        # mean_anlge_diff = direction_angle_camera  # TODO: adapt to also account for wheel speed
        # self.kf.x[2] += mean_anlge_diff

        self.kf.update(np.array([position_camera_est[0], position_camera_est[1], direction_angle_camera, 1]))#var_angle_diff))

        # Predict the next state
        print("predicted x", self.kf.x)
        print("direction_angle_camera           ", direction_angle_camera)

        print("camera diff", direction_angle_camera - self.kf.x[2])
        print("wheel diff ", w * dt)
        print("right_wheel_speed:", right_wheel_speed)
        print("left_wheel_speed:", left_wheel_speed)
        self.kf.x[2] = direction_angle_camera
        # print("updated x_after pred", self.kf.x)

    def get_location_est(self):
        return int(self.kf.x[0]), int(self.kf.x[1])

    def get_angle_est(self):
        return self.kf.x[2] % (2 * math.pi)

    def get_direction_est(self):
        # Convert angle from degrees to radians
        angle = self.get_angle_est()
        angle_rad = angle  # * math.pi / 180.0

        # Compute the direction vector
        direction_x = math.cos(angle_rad)
        direction_y = math.sin(angle_rad)

        return direction_x, direction_y

    def set_thymio_kidnap_location(self, position_thymio_camera_est):
        self.kf.x = np.array([position_thymio_camera_est[0], position_thymio_camera_est[1], 0.0])
        print("kidnap x", self.kf.x)
