import numpy as np
import math
from filterpy.kalman import KalmanFilter


class ThymioKalmanFilter:
    def __init__(self, position_thymio_camera_est):
        self.kf = KalmanFilter(dim_x=3, dim_z=2)

        # Initial state (x, y, angle)
        self.kf.x = np.array([position_thymio_camera_est[0], position_thymio_camera_est[1], 0.0])

        print("initial x", self.kf.x)

        # State transition matrix
        self.kf.F = np.array([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 1]])

        # Measurement function
        self.kf.H = np.array([[1, 0, 0],
                              [0, 1, 0]])

        # Measurement noise covariance
        self.kf.R = np.array([[0.1, 0],
                              [0, 0.1]])

        # Process noise covariance
        self.kf.Q = np.array([[0.001, 0, 0],
                              [0, 0.001, 0],
                              [0, 0, 0.001]])

    # def update(self, position_camera_est, direction_camera_est, left_wheel_speed, right_wheel_speed):
    #     left_wheel_speed = left_wheel_speed / 10000.0
    #     right_wheel_speed = right_wheel_speed / 10000.0
    #
    #     # Update the state transition matrix based on wheel speeds
    #     dt = 1.0  # Time step (you may need to adjust this based on your system)
    #     v = (left_wheel_speed + right_wheel_speed) / 2.0
    #     w = (right_wheel_speed - left_wheel_speed) / 0.2  # Assuming wheelbase of 0.2 (you may need to adjust this)
    #
    #     self.kf.F[0, 2] = v * math.cos(self.kf.x[2]) * dt
    #     self.kf.F[1, 2] = v * math.sin(self.kf.x[2]) * dt
    #
    #     # Predict the next state
    #     self.kf.predict()
    #
    #     # Update the measurement based on the camera estimation
    #     self.kf.update(np.array([position_camera_est[0], position_camera_est[1]]))
    #
    #     print("updated x", self.kf.x)

    def update(self, position_camera_est, direction_camera_est, left_wheel_speed, right_wheel_speed):
        left_wheel_speed = left_wheel_speed / 10000.0
        right_wheel_speed = right_wheel_speed / 10000.0

        # Update the state transition matrix based on wheel speeds
        dt = 1.0  # Time step (you may need to adjust this based on your system)
        v = (left_wheel_speed + right_wheel_speed) / 2.0
        w = (right_wheel_speed - left_wheel_speed) / 0.2  # Assuming wheelbase of 0.2 (you may need to adjust this)

        self.kf.F[0, 2] = v * math.cos(self.kf.x[2]) * dt
        self.kf.F[1, 2] = v * math.sin(self.kf.x[2]) * dt

        # Update the measurement based on the camera estimation
        # measurement = np.array([position_camera_est[0], position_camera_est[1]])
        # self.kf.update(measurement)

        # Update the direction estimation based on camera estimation and wheel speed difference
        angle_diff = math.atan2(direction_camera_est[1], direction_camera_est[0])  # - self.kf.x[2]
        # angle_diff_wheels = w * dt
        var_angle_diff = angle_diff - self.kf.x[2]

        #
        # mean_anlge_diff = (angle_diff + angle_diff_wheels) / 2.0
        mean_anlge_diff = angle_diff  # TODO: adapt to also account for wheel speed

        self.kf.x[2] += mean_anlge_diff

        self.kf.update(np.array([position_camera_est[0], position_camera_est[1]], var_angle_diff))
        # self.kf.update(np.array([position_camera_est[0], position_camera_est[1]], mean_anlge_diff))

        print("camera est", position_camera_est, math.atan2(direction_camera_est[1], direction_camera_est[0]))
        print("updated x", self.kf.x)
        print("----")
        # Predict the next state
        self.kf.predict()
        self.kf.x[2] = angle_diff
        print("updated x_after pred", self.kf.x)

    def get_location_est(self):
        return int(self.kf.x[0]), int(self.kf.x[1])

    def get_angle_est(self):
        return self.kf.x[2]

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
