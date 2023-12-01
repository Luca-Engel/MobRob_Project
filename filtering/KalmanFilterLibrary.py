import numpy as np
import math
from filterpy.kalman import KalmanFilter

# seconds for 100 and -100 speed to make a 360 turn
MIN_THYMIO_360_TURN_PERIOD = 9

DT = 0.25


class ThymioKalmanFilter:
    """
        Kalman filter for the Thymio robot.
        The state vector is [x, y, theta, 1] where x and y are the position of the robot, theta is the angle of the robot
        and 1 is a constant to make the matrix multiplication work with additions to the elements.

        Attributes
        ----------
        kf : KalmanFilter : KalmanFilter
            The Kalman filter object.
        last_temp_direction_angle_camera : float
            The last angle of the direction vector from the camera's estimation.
        total_direction_angle_camera : float
            The total angle of the direction vector from the camera's estimation.
            This is used to have a smooth transition and not jump from 360 to 0 degrees or vice versa (as this
            would cause a large change in the angle and break the smooth prediction).
    """
    def __init__(self, position_thymio_camera_est, direction_thymio_camera_est):
        self.kf = None
        self.last_temp_direction_angle_camera = None
        self.total_direction_angle_camera = None

        self._initialize_kalman_filter(position_thymio_camera_est, direction_thymio_camera_est)

    def _initialize_kalman_filter(self, position_thymio_camera_est, direction_thymio_camera_est):
        """
        Initialize the Kalman filter.
        :param position_thymio_camera_est: estimated position of the Thymio from the camera, cannot be None
        :param direction_thymio_camera_est: estimated direction of the Thymio from the camera, cannot be None
        :return: None
        """
        self.kf = KalmanFilter(dim_x=4, dim_z=4)

        dir_x, dir_y = direction_thymio_camera_est
        angle = math.atan2(dir_y, dir_x)
        if angle < 0:
            angle += 2 * math.pi
        self.last_temp_direction_angle_camera = angle
        self.total_direction_angle_camera = angle

        # Initial state (x, y, angle)
        self.kf.x = np.array([position_thymio_camera_est[0], position_thymio_camera_est[1], angle, 1])

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
                              [0, 0, 0.1, 0],  # 0.001, 0],
                              [0, 0, 0, 0.001]])

        # Control input matrix should account for the position and orientation of the robot
        self.kf.B = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])  # TODO: check if it needs to be a 3x1 matrix

    def update(self, position_camera_est, direction_camera_est, left_wheel_speed, right_wheel_speed):
        """
        Update the Kalman filter based on the measurements from the camera and the wheel speeds.
        :param position_camera_est: estimated position of the Thymio from the camera, can be None (when camera is covered)
        :param direction_camera_est: estimated direction of the Thymio from the camera, can be None (when camera is covered)
        :param left_wheel_speed: turning speed of the left wheel
        :param right_wheel_speed: turning speed of the right wheel
        :return: None
        """

        v = (left_wheel_speed + right_wheel_speed) / 2.0

        right_wheel_speed = max(min(right_wheel_speed, 100), -100)
        left_wheel_speed = max(min(left_wheel_speed, 100), -100)

        period = 0
        if abs(right_wheel_speed - left_wheel_speed) > 0.5:  # avoid division by 0 and large values
            period = (200 / (
                    right_wheel_speed - left_wheel_speed)) * MIN_THYMIO_360_TURN_PERIOD  # positive sense is counterclockwise
        w = 0
        if period != 0:
            w = 2 * np.pi / period

        self.kf.F[2, 3] = w  # * 0.1 #DT  # becomes theta_k+1 = theta_k + w * DT

        if abs(v) < 20:
            # Pure rotation without translation, i.e., the thymio is not moving forward or backward
            # --> keep the postion the same
            self.kf.F[0, 3] = 1
            self.kf.F[1, 3] = 1
        else:
            # Translation with rotation
            self.kf.F[0, 3] = v * math.cos(self.kf.x[2]) * DT  # becomes x_k+1 = x_k + v * cos(theta) * DT
            self.kf.F[1, 3] = v * math.sin(self.kf.x[2]) * DT  # becomes y_k+1 = y_k + v * sin(theta) * DT

        # update last element of x to guarantee that it is always 1
        self.kf.x[3] = 1

        self.kf.predict()

        # Get the angle of the direction vector from the camera's estimation
        direction_angle_camera = math.atan2(direction_camera_est[1], direction_camera_est[0])
        direction_angle_camera = direction_angle_camera % (2 * math.pi)

        # always add the change in angle from last time to this time to the "total" angle
        angle_change = direction_angle_camera - self.last_temp_direction_angle_camera

        # since only small angle changes expected, we can assume that the change is between -pi and pi
        # --> if not, we need to wrap around (as there is a jump from 360 to 0 degrees or vice versa)
        if angle_change > np.pi:
            angle_change = angle_change - 2 * np.pi
        elif angle_change < -np.pi:
            angle_change = angle_change + 2 * np.pi

        self.total_direction_angle_camera = self.total_direction_angle_camera + angle_change
        self.last_temp_direction_angle_camera = direction_angle_camera

        self.kf.update(np.array(
            [position_camera_est[0], position_camera_est[1], self.total_direction_angle_camera, 1]))

    def get_location_est(self):
        """
        Get the estimated location of the Thymio.
        :return: tuple of ints (x, y)
        """
        return int(self.kf.x[0]), int(self.kf.x[1])

    def get_angle_rad_est(self):
        """
        Get the estimated angle of the Thymio in radians.
        :return: float between 0 and 2pi
        """
        return self.kf.x[2] % (2 * math.pi)

    def get_direction_est(self):
        """
        Get the estimated direction of the Thymio.
        :return: tuple of floats (x, y)
        """
        angle_rad = self.get_angle_rad_est()
        return self._direction_est_for_angle(angle_rad)

    def _direction_est_for_angle(self, angle_rad):
        """
        Get the direction vector for the given angle.
        :param angle_rad: angle in radians
        :return: tuple of floats (x, y)
        """
        # Compute the direction vector
        direction_x = math.cos(angle_rad)
        direction_y = math.sin(angle_rad)

        return direction_x, direction_y

    def set_thymio_kidnap_location(self, position_thymio_camera_est, direction_thymio_camera_est):
        """
        Set the location of the Thymio to the given position and direction after kidnapping.
        :param position_thymio_camera_est: new estimated position of the Thymio from the camera
        :param direction_thymio_camera_est: new estimated direction of the Thymio from the camera
        :return: None
        """
        # dir_x, dir_y = direction_thymio_camera_est
        # angle = math.atan2(dir_y, dir_x)
        # self.kf = KalmanFilter
        #
        # self.kf.x = np.array([position_thymio_camera_est[0], position_thymio_camera_est[1], 0.0, 1])
        # print("kidnap x", self.kf.x)
        self._initialize_kalman_filter(position_thymio_camera_est, direction_thymio_camera_est)
