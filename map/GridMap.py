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

    def __init__(self, width, height, object_detector):
        self.width = width
        self.height = height
        self.grid = np.full((height, width), CellType.FREE, dtype='object')
        self.object_detector = object_detector

    def update_grid(self):
        contours, binary_image, frame_with_objects = self.object_detector.detect_objects()

        print("binary image shape:", binary_image.shape)
        print("binary image element types:", binary_image.dtype)
        # if ids is not None:
        #     for i, corner in enumerate(corners):
        #         if ids[i] == 4:
        #             # ArUco marker with id 4
        #             self._update_grid_with_marker(corner, CellType.MARKER)
        #         else:
        #             # ArUco markers with ids 0-3
        #             self._update_grid_with_marker(corner, CellType.OBJECT)

        image_height = len(binary_image)
        image_width = len(binary_image[0])

        for row_pixel in range(image_height):
            for column_pixel in range(image_width):
                if binary_image[row_pixel][column_pixel] < 100: # no object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.FREE)
                else: # object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.OBJECT)

    def _update_grid_with_object(self, row_pixel, column_pixel, image_width, image_height, value):
        x = int(column_pixel / image_width * self.width)
        y = int(row_pixel / image_height * self.width)

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
        grid_image = np.vectorize((lambda x: 255 if x == CellType.FREE else 0))(self.grid).astype(np.uint8)

        cv2.imshow("grid map", grid_image)


if __name__ == "__main__":
    # Sample usage:
    width, height = 100, 100
    webcam = WebcamFeed()
    aruco_detector = ArUcoMarkerDetector(webcam)
    object_detector = ObjectDetector(aruco_detector)
    grid_map = GridMap(width, height, object_detector)

    while True:
        object_detector.detect_objects()
        grid_map.update_grid()
        grid_map.display_grid_as_image()

        if webcam.user_has_quit():
            webcam.release_resources()
            break

    print("Done.")
