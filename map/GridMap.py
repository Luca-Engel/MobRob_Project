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

    def __init__(self, width, height, thymio_marker_id = 0, goal_marker_id = 1):
        self.width = width
        self.height = height
        self.thymio_marker_id = thymio_marker_id
        self.goal_marker_id = goal_marker_id

        self.webcam = WebcamFeed()
        self.aruco_detector = ArUcoMarkerDetector(self.webcam)
        self.object_detector = ObjectDetector(self.aruco_detector)

        self.grid = np.full((height, width), CellType.FREE, dtype='object')
        self.update_grid()

        color_mapping = {
            CellType.FREE: (255, 255, 255),  # White
            CellType.THYMIO: (0, 255, 0),  # Green
            CellType.OBJECT: (0, 0, 0),  # Black
            CellType.MARKER: (0, 0, 255),  # Red
            CellType.GOAL: (255, 0, 0)  # Blue
        }
        # self.grid_image = np.vectorize(lambda x: 255 if x == CellType.FREE else (150 if x == CellType.THYMIO else 0))(self.grid).astype(np.uint8)
        self.grid_image = np.vectorize(lambda x: color_mapping.get(x, (0, 0, 0)))(self.grid)

        self.grid_image = np.stack(self.grid_image, axis=-1)

        # Convert to uint8 for imshow
        self.grid_image = self.grid_image.astype(np.uint8)


    def update_grid(self):
        contours, binary_image, frame_with_objects, corners, ids = self.object_detector.detect_objects()
        print("frame types: ", frame_with_objects.dtype)

        print(binary_image)
        image_height = len(binary_image)
        image_width = len(binary_image[0])

        print("width: ", image_width)
        print("height: ", image_height)
        for row_pixel in range(image_height):
            for column_pixel in range(image_width):
                if binary_image[row_pixel][column_pixel] < 100: # no object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.FREE)
                else: # object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.OBJECT)

        if self.thymio_marker_id in ids:
            print("thymio check corners shape:", corners.shape)
            corners_for_thymio = corners[np.where(ids == self.thymio_marker_id)[0]][0]
            self._update_grid_with_marker(corners_for_thymio, CellType.THYMIO, len(binary_image[0]), len(binary_image))

        if self.goal_marker_id in ids:
            print("thymio check corners shape:", corners.shape)
            corners_for_thymio = corners[np.where(ids == self.goal_marker_id)[0]][0]
            self._update_grid_with_marker(corners_for_thymio, CellType.GOAL, len(binary_image[0]), len(binary_image))

    def _update_grid_with_object(self, row_pixel, column_pixel, image_width, image_height, value):
        # print("row_pixel: ", row_pixel)
        # print("column_pixel: ", column_pixel)
        x = int(column_pixel / image_width * self.width)
        y = int(row_pixel / image_height * self.height)

        self.grid[y, x] = value


    def _update_grid_with_marker(self, corner, value, video_feed_width, video_feed_height):
        thymio_marker_corners = corner[0]
        # find centroid of the corners in grid coordinates
        x, y = self._convert_to_grid_indices(thymio_marker_corners, video_feed_width, video_feed_height)
        if value == CellType.GOAL:
            self.goal_location = (x, y)

        # Update grid with value in a 20x20 square around the marker
        for i in range(x - 10, x + 10):
            for j in range(y - 10, y + 10):
                if 0 <= i < self.width and 0 <= j < self.height:
                    # draw circle in grid
                    if (i - x) ** 2 + (j - y) ** 2 <= 10 ** 2:
                        self.grid[j, i] = value

    def _convert_to_grid_indices(self, corners, video_feed_width, video_feed_height):
        centroid = np.mean(corners, axis=0)

        print("centroid: ", centroid)
        print(corners.shape)
        print(centroid.shape)
        # Convert corner coordinates to grid indices
        x = int(centroid[0] / video_feed_width * self.width)
        y = int(centroid[1] / video_feed_height * self.height)
        return x, y

    def display_grid_as_image(self):
        cv2.imshow("grid map", self.grid_image)

    def display_feed(self):
        _ = self.object_detector.detect_objects()

    def user_has_quit(self):
        return self.webcam.user_has_quit()

    def release_resources(self):
        self.webcam.release_resources()
        cv2.destroyAllWindows()

    def get_goal_location(self):
        return self.goal_location


if __name__ == "__main__":
    # Sample usage:
    width, height = 320, 240
    thymio_marker_id = 0
    goal_marker_id = 1

    grid_map = GridMap(width, height, thymio_marker_id, goal_marker_id)

    while True:
        grid_map.display_feed()
        grid_map.display_grid_as_image()

        if grid_map.user_has_quit():
            grid_map.release_resources()
            break

    print("Done.")
