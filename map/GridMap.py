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

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.full((height, width), CellType.FREE, dtype='object')

        self.webcam = WebcamFeed()
        self.aruco_detector = ArUcoMarkerDetector(self.webcam)
        self.object_detector = ObjectDetector(self.aruco_detector)

        self.update_grid()
        self.grid_image = np.vectorize((lambda x: 255 if x == CellType.FREE else 0))(self.grid).astype(np.uint8)

    def update_grid(self):
        contours, binary_image, frame_with_objects = self.object_detector.detect_objects()

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

    def _update_grid_with_object(self, row_pixel, column_pixel, image_width, image_height, value):
        # print("row_pixel: ", row_pixel)
        # print("column_pixel: ", column_pixel)
        x = int(column_pixel / image_width * self.width)
        y = int(row_pixel / image_height * self.height)

        self.grid[y, x] = value


    def _update_grid_with_marker(self, corner, value):
        # Convert corner coordinates to grid indices
        x, y = self._convert_to_grid_indices(corner)

        # Update the grid with the specified value in a square region around the marker
        for i in range(x - 10, x + 10):
            for j in range(y - 10, y + 10):
                if 0 <= i < self.width and 0 <= j < self.height:
                    self.grid[j, i] = value

    def _convert_to_grid_indices(self, corner):
        # Convert corner coordinates to grid indices
        x = int((corner[0, 0] + corner[2, 0]) / 2 / self.width)
        y = int((corner[0, 1] + corner[2, 1]) / 2 / self.height)
        return x, y

    def display_grid_as_image(self):
        cv2.imshow("grid map", self.grid_image)

    def display_feed(self):
        contours, binary_image, frame_with_objects = self.object_detector.detect_objects()

    def user_has_quit(self):
        return self.webcam.user_has_quit()

    def release_resources(self):
        self.webcam.release_resources()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Sample usage:
    width, height = 300, 250

    grid_map = GridMap(width, height)

    while True:
        grid_map.display_feed()
        grid_map.display_grid_as_image()

        if grid_map.user_has_quit():
            grid_map.release_resources()
            break

    print("Done.")
