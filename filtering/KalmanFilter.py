# import numpy as np
# import math
#
#
# class KalmanFilter:
#     def __init__(self, position_thymio_camera_est):
#         pos_x, pos_y = position_thymio_camera_est
#         self.x = np.array([pos_x, pos_y, 0]) #np.zeros(4)  # state vector (x, y, rotation)
#         self.P = np.zeros((3, 3))  # state covariance matrix
#         self.F = np.eye(3)  # state transition matrix
#         self.Q = np.diag([10, 10, 0.4])  # process covariance matrix
#         self.H = np.eye(3)  # measurement matrix
#         self.R = np.diag([3, 3, 0.1])#1 #np.ones(4)  # measurement covariance matrix
#
#
#         print("initial x", self.x)
#
#     def _predict(self, left_wheel_speed, right_wheel_speed, dt):
#         # Update state transition matrix based on the time step (dt)
#         # self.F[0, 2] = dt * math.cos(self.x[2])
#         # self.F[0, 3] = -dt * math.sin(self.x[2])
#         # self.F[1, 2] = dt * math.sin(self.x[2])
#         # self.F[1, 3] = dt * math.cos(self.x[2])
#         self.F[0, 2] = dt * math.cos(self.x[2])
#         self.F[1, 2] = dt * math.sin(self.x[2])
#
#         # Update state prediction based on wheel speeds
#         v = (left_wheel_speed + right_wheel_speed) / 2.0
#         omega = (right_wheel_speed - left_wheel_speed) / 2.0
#         self.x[0] += dt * v * math.cos(self.x[2])
#         self.x[1] += dt * v * math.sin(self.x[2])
#         self.x[2] += dt * omega
#
#         # Predict the state covariance matrix
#         self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
#
#
#     def update(self, position_camera_est, direction_camera_est, left_wheel_speed, right_wheel_speed):
#         left_wheel_speed = left_wheel_speed / 10000.0
#         right_wheel_speed = right_wheel_speed / 10000.0
#
#         # print("update called!")
#         # print("position_camera_est", position_camera_est)
#         # print("direction_camera_est", direction_camera_est)
#         # print("left_wheel_speed", left_wheel_speed)
#         # print("right_wheel_speed", right_wheel_speed)
#
#         # Kalman gain calculation
#         # K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(np.dot(np.dot(self.H, self.P), self.H.T) + self.R))
#         s = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
#         # print("s", s)
#         K = np.dot(np.dot(self.P, self.H.T), 1/s)
#         # K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(s))
#         # K = np.dot(np.dot(self.P, self.H.T), 1/s)
#
#         direction_camera_angle_est = math.atan2(direction_camera_est[1], direction_camera_est[0])*(np.pi/180)
#         # print("direction_camera_angle_est", direction_camera_angle_est)
#         # Measurement residual
#         z = np.array([position_camera_est[0], position_camera_est[1], direction_camera_angle_est])
#         y = z - np.dot(self.H, self.x)
#
#         # Update state estimate and covariance
#         # print("K", K.shape)
#         # print("y", y.shape)
#         self.x = self.x + np.dot(K, y) # calculation of new x is wrong!!!
#         self.P = np.dot((np.eye(3) - np.dot(K, self.H)), self.P)
#
#         print("updated x", self.x)
#
#         # Perform prediction step for the next iteration
#         dt = 0.1  # TODO: adjust to the actual time step
#
#         self._predict(left_wheel_speed, right_wheel_speed, dt)
#
#
#     def get_location_est(self):
#         # print("update called before get pos?")
#
#         return int(self.x[0]), int(self.x[1])
#
#     def get_angle_est(self):
#         return self.x[2]
#
#     def get_direction_est(self):
#         # Convert angle from degrees to radians
#         angle = self.x[2]
#         angle_rad = angle * math.pi / 180.0
#
#         # Compute the direction vector
#         direction_x = math.cos(angle_rad)
#         direction_y = math.sin(angle_rad)
#
#         return (direction_x, direction_y)