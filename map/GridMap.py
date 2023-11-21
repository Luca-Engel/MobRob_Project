import cv2
import numpy as np
import enum

from opencv.webcam_input import WebcamFeed
from aruco.marker_recognition import ArUcoMarkerDetector
from opencv.object_recognition import ObjectDetector


class CellType(enum.Enum):
    FREE = '_'
    OBJECT = 'x'
    MARKER = 'm'
    THYMIO = 't'
    GOAL = 'g'

    def __str__(self):
        return self.value


class GridMap:

    def __init__(self, width, height, thymio_marker_id=0, goal_marker_id=1):
        self.width = width
        self.height = height
        self.thymio_marker_id = thymio_marker_id
        self.goal_marker_id = goal_marker_id
        self.thymio_corners = None
        self.goal_corners = None
        self.thymio_location = None
        self.goal_location = None
        self.grid_image_is_up_to_date = False

        self.color_mapping = {
            CellType.FREE: (255, 255, 255),  # White
            CellType.THYMIO: (0, 255, 0),  # Green
            CellType.OBJECT: (0, 0, 0),  # Black
            CellType.MARKER: (0, 0, 255),  # Red
            CellType.GOAL: (255, 0, 0)  # Blue
        }

        self.webcam = WebcamFeed()
        self.aruco_detector = ArUcoMarkerDetector(self.webcam)
        self.object_detector = ObjectDetector(self.aruco_detector)

        while True:
            if cv2.waitKey(1) & 0xFF == ord('b'):
                break
            cv2.imshow("Press b to initialize Map", self.webcam.single_capture_and_display())

        self.grid = np.full((height, width), CellType.FREE, dtype='object')
        self.update_grid()
        self._compute_grid_image()
        self.grid_image_is_up_to_date = True

    def _compute_grid_image(self):

        self.grid_image = np.vectorize(lambda x: self.color_mapping.get(x, (0, 0, 0)))(self.grid)
        self.grid_image = np.stack(self.grid_image, axis=-1)
        # Convert to uint8 for imshow
        self.grid_image = self.grid_image.astype(np.uint8)

    def update_grid(self):
        contours, binary_image, frame_with_objects, corners, ids = self.object_detector.detect_objects()

        image_height = len(binary_image)
        image_width = len(binary_image[0])

        for row_pixel in range(image_height):
            for column_pixel in range(image_width):
                if binary_image[row_pixel][column_pixel] < 100:  # no object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.FREE)
                else:  # object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.OBJECT)

        self._update_thymio_grid_location(binary_image, corners, ids)
        self._update_goal_grid_location(binary_image, corners, ids)

    def update_goal_and_thymio_grid_location(self):
        contours, binary_image, frame_with_objects, corners, ids = self.object_detector.detect_objects()

        self._update_thymio_grid_location(binary_image, corners, ids)
        self._update_goal_grid_location(binary_image, corners, ids)

    def _update_goal_grid_location(self, binary_image, corners, ids):
        if ids is not None and self.goal_marker_id in ids:
            corners_for_thymio = corners[np.where(ids == self.goal_marker_id)[0]][0]
            if self.goal_corners is not None and self.goal_corners == corners_for_thymio:
                return
            self._update_grid_with_marker(corners_for_thymio, CellType.GOAL, len(binary_image[0]), len(binary_image), self.goal_location)
            self.grid_image_is_up_to_date = False

    def _update_thymio_grid_location(self, binary_image, corners, ids):
        if ids is not None and self.thymio_marker_id in ids:
            corners_for_thymio = corners[np.where(ids == self.thymio_marker_id)[0]][0]
            if self.thymio_corners is not None and self.thyio_corners == corners_for_thymio:
                return
            self._update_grid_with_marker(corners_for_thymio, CellType.THYMIO, len(binary_image[0]), len(binary_image), self.thymio_location)
            self.grid_image_is_up_to_date = False

    def _update_grid_with_object(self, row_pixel, column_pixel, image_width, image_height, value):
        x = int(column_pixel / image_width * self.width)
        y = int(row_pixel / image_height * self.height)

        self.grid[y, x] = value

    def _update_grid_with_marker(self, corner, value, video_feed_width, video_feed_height, last_location=None):
        marker_corners = corner[0]
        # find centroid of the corners in grid coordinates
        x, y = self._convert_to_grid_indices(marker_corners, video_feed_width, video_feed_height)
        if value == CellType.GOAL:
            self.goal_location = (x, y)
        elif value == CellType.THYMIO:
            self.thymio_location = (x, y)

        if last_location is not None:
            self._draw_marker_circle(CellType.FREE, last_location[0], last_location[1])
        self._draw_marker_circle(value, x, y)

    def _draw_marker_circle(self, value, x, y):
        # Update grid with value in a 20x20 square around the marker
        for i in range(x - 10, x + 10):
            for j in range(y - 10, y + 10):
                if 0 <= i < self.width and 0 <= j < self.height:
                    # draw circle in grid
                    if (i - x) ** 2 + (j - y) ** 2 <= 10 ** 2:
                        self.grid[j, i] = value

    def _convert_to_grid_indices(self, corners, video_feed_width, video_feed_height):
        centroid = np.mean(corners, axis=0)

        # Convert corner coordinates to grid indices
        x = int(centroid[0] / video_feed_width * self.width)
        y = int(centroid[1] / video_feed_height * self.height)
        return x, y

    def display_grid_as_image(self):
        """
        Displays the current grid as an image
        :return: None
        """
        if self.grid_image_is_up_to_date:
            cv2.imshow("grid map", self.grid_image)
        else:
            self._compute_grid_image()
            cv2.imshow("grid map", self.grid_image)
            self.grid_image_is_up_to_date = True

    def display_feed(self):
        """
        Displays the current webcam feed
        :return:
        """
        contours, binary_image, frame_with_objects, corners, ids = self.object_detector.detect_objects()

        cv2.imshow("Current Feed", frame_with_objects)

    def user_has_quit(self):
        """
        Returns True if the user has quit the program
        :return: bool
        """
        return self.webcam.user_has_quit()

    def release_resources(self):
        """
        Releases the resources used by the webcam
        :return: None
        """
        self.webcam.release_resources()
        cv2.destroyAllWindows()

    def get_goal_location(self):
        """
        Returns the goal location in grid coordinates
        :return: (x, y) tuple
        """
        if self.goal_location is None:
            throw("Goal location not found")

        return self.goal_location

    def get_thymio_location(self):
        """
        Returns the thymio location in grid coordinates
        :return: (x, y) tuple
        """
        if self.thymio_location is None:
            throw("Thymio location not found")

        return self.thymio_location


if __name__ == "__main__":
    # Sample usage:
    width, height = 320, 240
    thymio_marker_id = 4
    goal_marker_id = 5

    grid_map = GridMap(width, height, thymio_marker_id, goal_marker_id)

    while True:
        # TODO: Add code to update grid map and delete set the last location of the thymio and goal to FREE
        # if they change location
        grid_map.update_goal_and_thymio_grid_location()

        grid_map.display_feed()
        grid_map.display_grid_as_image()

        if grid_map.user_has_quit():
            grid_map.release_resources()
            break

    print("Done.")
