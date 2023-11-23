import enum

import cv2
import numpy as np

from aruco.marker_recognition import ArUcoMarkerDetector
from opencv.object_recognition import ObjectDetector
from opencv.webcam_input import WebcamFeed

# Width/2 of the thymio in grid cells
THYMIO_HALF_SIZE = 12

# Any island with a radius of 2 cells or less will be removed
ISLAND_REMOVAL_RADIUS = 2


class CellType(enum.Enum):
    FREE = '_'
    OBJECT = 'x'
    OBJECT_SIZE_INCREASE = 'o'
    MARKER = 'm'
    THYMIO = 't'
    GOAL = 'g'
    PATH = 'p'

    def __str__(self):
        return self.value


class GridMap:
    def __init__(self, width=160, height=120, thymio_marker_id=4, goal_marker_id=5, load_from_file=None):
        self._width = width
        self._height = height
        self._thymio_marker_id = thymio_marker_id
        self._goal_marker_id = goal_marker_id
        self._thymio_corners = None
        self._goal_corners = None
        self._thymio_location = None
        self._thymio_location_prev_grid_value = None
        self._thymio_direction = None
        self._goal_location = None
        self.path = None
        self._grid_image_is_up_to_date = False

        self._color_mapping = {
            CellType.FREE: (255, 255, 255),  # White
            CellType.THYMIO: (0, 255, 0),  # Green
            CellType.OBJECT: (0, 0, 0),  # Black
            CellType.OBJECT_SIZE_INCREASE: (100, 100, 100),  # Grey
            CellType.MARKER: (0, 0, 255),  # Red
            CellType.GOAL: (255, 0, 0),  # Blue
            CellType.PATH: (0, 0, 255)  # Yellow
        }

        self._webcam = WebcamFeed(load_from_file=load_from_file)
        self._aruco_detector = ArUcoMarkerDetector(self._webcam)
        self._object_detector = ObjectDetector(self._aruco_detector)
        self._image_feed_width = None
        self._image_feed_height = None

        while True:
            if cv2.waitKey(1) & 0xFF == ord('b'):
                break
            cv2.imshow("Press b to initialize Map", self._webcam.single_capture_and_display())

        self._grid = np.full((height, width), CellType.FREE, dtype='object')
        self._previous_grid = None
        self.update_grid()

        self._remove_island_objects_from_grid(radius=ISLAND_REMOVAL_RADIUS)
        self._increase_object_size(radius=THYMIO_HALF_SIZE)
        self._remove_island_objects_from_grid(radius=ISLAND_REMOVAL_RADIUS)

        self._compute_grid_image()
        self._grid_image_is_up_to_date = True


    def _increase_object_size(self, radius=1):
        """
        Increases the size of the objects in the grid by the given radius (of the thymio)
        :param radius: the radius of the object
        :return: None
        """
        new_grid = self._grid.copy()

        for y in range(self._height):
            for x in range(self._width):
                if self._grid[y, x] == CellType.OBJECT:
                    for k in range(-radius, radius + 1):
                        for l in range(-radius, radius + 1):
                            if (0 <= y + k < self._height
                                    and 0 <= x + l < self._width
                                    and (l) ** 2 + (k) ** 2 <= radius ** 2
                            ):
                                # and not (-radius < k < radius and -radius < l < radius)):
                                if self._grid[y + k, x + l] != CellType.OBJECT:
                                    new_grid[y + k, x + l] = CellType.OBJECT_SIZE_INCREASE
        self._grid = new_grid
        self._previous_grid = self._grid.copy()
        self._grid_image_is_up_to_date = False
        self.update_grid()

    def _remove_island_objects_from_grid(self, radius=1):
        """
        Removes objects that are islands of max size radius
        :param radius: the max radius of the island
        :return: None
        """
        for y in range(self._height):
            for x in range(self._width):
                if self._grid[y, x] == CellType.OBJECT:
                    if self._check_if_cells_are_island(x, y, radius):
                        self._grid[y, x] = CellType.FREE
        self._grid_image_is_up_to_date = False

    def _check_if_cells_are_island(self, x, y, radius):
        """
        Checks if the cells around the given cell island of size radius is an island
        :param x: the x coordinate of the cell
        :param y: the y coordinate of the cell
        :param radius: the radius of the island
        :return: True if the cells within the radius form an island, False otherwise
        """
        if (self._grid[y, x] != CellType.OBJECT):
            print("error: cell is not an object")
            print("(x, y): ", x, y)
        count = 0
        for k in range(-radius, radius + 1):
            for l in range(-radius, radius + 1):
                if (0 <= y + k < self._height
                        and 0 <= x + l < self._width
                        and not (-radius < k < radius and -radius < l < radius)):
                    if self._grid[y + k, x + l] == CellType.OBJECT:
                        count += 1
        return count < 2  # i.e., no neighbours of the island within the radius are objects -> island

    def _compute_grid_image(self, scale_factor=5):
        """
        Computes the grid image corresponding to the 2d grid
        :param scale_factor: the scale factor to increase the size of the displayed grid image
        :return: None
        """

        self.grid_image = np.array(np.vectorize(lambda x: self._color_mapping.get(x, (0, 0, 0)))(self._grid))
        self.grid_image = np.stack(self.grid_image, axis=-1)

        height, width = self.grid_image.shape[:2]

        # Create a new image with double the dimensions
        resized_image = np.zeros((scale_factor * height, scale_factor * width, 3), dtype=np.uint8)

        # Copy each pixel value to the corresponding 2x2 block in the new image
        for i in range(height):
            for j in range(width):
                resized_image[scale_factor * i:scale_factor * (i + 1), scale_factor * j:scale_factor * (j + 1),
                :] = self.grid_image[i, j, :]
        self.grid_image = np.array(resized_image)
        # Convert to uint8 for imshow
        self.grid_image = self.grid_image.astype(np.uint8)

    def update_grid(self):
        """
        Updates the grid with the current webcam feed.
        Automatically removes the markers from the grid.
        Places the thymio and goal markers on the grid.
        :return: None
        """

        contours, binary_image, frame_with_objects, corners, ids = self._object_detector.detect_objects()

        marker_width = 0
        if corners is not None and len(corners) > 0:
            marker_width = corners[0][0][1][0] - corners[0][0][0][0]

        image_height = len(binary_image)
        image_width = len(binary_image[0])
        self._image_feed_width = image_width
        self._image_feed_height = image_height

        for row_pixel in range(image_height):
            for column_pixel in range(image_width):
                if binary_image[row_pixel][column_pixel] < 100:  # no object
                    # self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.FREE)
                    continue
                else:  # object
                    self._update_grid_with_object(row_pixel, column_pixel, image_width, image_height, CellType.OBJECT)

        self._remove_markers_as_objects_from_grid(binary_image, corners, ids)

        self._update_thymio_grid_location_and_direction(binary_image, corners, ids)
        self._update_goal_grid_location(binary_image, corners, ids)

    def _remove_markers_as_objects_from_grid(self, binary_image, corners, ids):
        """
        Removes the markers from the grid based on the corner locations of the markers
        :param binary_image: the binary image
        :param corners: the corners of the markers
        :param ids: the ids of the markers
        :return: None
        """

        if ids is not None:
            for c in corners:
                corner = c[0]

                x_coords, y_coords = corner[:, 0], corner[:, 1]

                # Get the minimum and maximum x, y coordinates to define the bounding box
                min_x, max_x = int(min(x_coords)), int(max(x_coords))
                min_y, max_y = int(min(y_coords)), int(max(y_coords))

                # Iterate through every pixel in the bounding box
                for x_image in range(min_x, max_x + 1):
                    for y_image in range(min_y, max_y + 1):
                        # TODO: decide whether or not to store the markers location on grid as MARKER or FREE
                        self._update_grid_with_object(y_image, x_image, len(binary_image[0]), len(binary_image),
                                                      CellType.FREE)  # CellType.MARKER)

                self._grid_image_is_up_to_date = False

    def update_goal_and_thymio_grid_location(self):
        """
        Updates the grid with the current webcam feed.
        :return: None
        """
        contours, binary_image, frame_with_objects, corners, ids = self._object_detector.detect_objects()

        self._update_thymio_grid_location_and_direction(binary_image, corners, ids)
        self._update_goal_grid_location(binary_image, corners, ids)

    def _update_goal_grid_location(self, binary_image, corners, ids):
        """
        Updates the goal location in the grid
        :param binary_image: the binary image
        :param corners: the corners of the markers
        :param ids: the ids of the markers
        :return: None
        """
        if ids is not None and self._goal_marker_id in ids:
            corners_for_goal = corners[np.where(ids == self._goal_marker_id)[0]][0]
            if self._goal_corners is not None and self._goal_corners == corners_for_goal:
                return
            self._update_grid_with_marker(corners_for_goal, CellType.GOAL, len(binary_image[0]), len(binary_image),
                                          self._goal_location)
            self._grid_image_is_up_to_date = False

    def _update_thymio_grid_location_and_direction(self, binary_image, corners, ids):
        """
        Updates the thymio location and its facing direction in the grid
        :param binary_image: the binary image
        :param corners: the corners of the markers
        :param ids: the ids of the markers
        :return: None
        """
        if ids is not None and self._thymio_marker_id in ids:
            corners_for_thymio = corners[np.where(ids == self._thymio_marker_id)[0]][0]
            if (self._thymio_corners is not None) and np.array_equal(self._thymio_corners, corners_for_thymio):
                return

            self._thymio_corners = corners_for_thymio
            self._update_thymio_direction()
            self._update_grid_with_marker(corners_for_thymio, CellType.THYMIO, len(binary_image[0]), len(binary_image),
                                          self._thymio_location)
            self._grid_image_is_up_to_date = False

    def _update_grid_with_object(self, row_pixel, column_pixel, image_width, image_height, value):
        """
        Updates the grid with the given value at the given pixel location
        :param row_pixel: row pixel location
        :param column_pixel: column pixel location
        :param image_width: width of the image
        :param image_height: height of the image
        :param value: value to update the grid with
        :return: None
        """
        x = int(column_pixel / image_width * self._width)
        y = int(row_pixel / image_height * self._height)

        if 0 <= x < self._width and 0 <= y < self._height:
            # self.grid[y, x] = value
            self._grid[y, x] = self._previous_grid[y, x] if self._previous_grid is not None else value

    def _update_grid_with_marker(self, corner, value, video_feed_width, video_feed_height, last_location=None):
        """
        Updates the grid with the given value at the given marker location
        :param corner: the corners delimiting the marker
        :param value: the value to update the grid with
        :param video_feed_width: width of the video feed
        :param video_feed_height: height of the video feed
        :param last_location: last location of the thymio if it exists
        :return: None
        """
        marker_corners = corner[0]
        # find centroid of the corners in grid coordinates
        x, y = self._convert_to_centroid_grid_indices(marker_corners, video_feed_width, video_feed_height)

        # update the last location of the thymio or goal to the last value / FREE, respectively
        if last_location is not None:
            self._draw_marker_circle(
                self._thymio_location_prev_grid_value if value == CellType.THYMIO else CellType.FREE,
                last_location[0], last_location[1])

        if value == CellType.GOAL:
            self._goal_location = (x, y)
        elif value == CellType.THYMIO:
            if 0 <= x < self._width and 0 <= y < self._height:
                self._thymio_location = (x, y)
                self._thymio_location_prev_grid_value = self._grid[y, x]

        self._draw_marker_circle(value, x, y)

    def _draw_marker_circle(self, value, x, y):
        """
        Draws a circle around the marker in the grid
        :param value: the value to update the grid with
        :param x: x grid coordinate of the marker
        :param y: y grid coordinate of the marker
        :return: None
        """
        # Update grid with value in a 20x20 square around the marker
        # for i in range(x - 10, x + 10):
        #     for j in range(y - 10, y + 10):
        #         if 0 <= i < self.width and 0 <= j < self.height:
        #             # draw circle in grid
        #             if (i - x) ** 2 + (j - y) ** 2 <= 10 ** 2:
        #                 self.grid[j, i] = value
        if 0 <= x < self._width and 0 <= y < self._height:
            self._grid[y, x] = value

    def _convert_to_centroid_grid_indices(self, corners, video_feed_width, video_feed_height):
        """
        Converts the centroid of the given corners to grid indices
        :param corners: the corners of the marker
        :param video_feed_width: width of the video feed
        :param video_feed_height: height of the video feed
        :return: (x, y) tuple delimiting the centroid in grid coordinates
        """
        centroid = np.mean(corners, axis=0)

        # Convert corner coordinates to grid indices
        x = int(centroid[0] / video_feed_width * self._width)
        y = int(centroid[1] / video_feed_height * self._height)
        return x, y

    def display_grid_as_image(self):
        """
        Displays the current grid as an image
        :return: None
        """
        if self._grid_image_is_up_to_date:
            cv2.imshow("grid map", self.grid_image)
        else:
            print("recomputing the grid image")
            self._compute_grid_image()
            cv2.imshow("grid map", self.grid_image)
            self._grid_image_is_up_to_date = True

        # thymio_location = self.get_thymio_grid_coordinates()
        # goal_location = self.get_goal_grid_coordinates()
        # thymio_location = self.get_thymio_location()
        # goal_location = self.get_thymio_location()
        #
        # new_image=cv2.arrowedLine(self.grid_image, thymio_location, goal_location, (0, 255, 255), 2)
        # cv2.imshow("image with thymio and goal location", new_image)

    def set_path(self, path):
        """
        Sets the path in the grid
        :param path: list of (x, y) tuples
        :return: None
        """
        for x, y in path:
            self._grid[y, x] = CellType.PATH

        self.path = path
        self._grid_image_is_up_to_date = False

    def display_feed(self):
        """
        Displays the current webcam feed
        :return: None
        """
        contours, binary_image, frame_with_objects, corners, ids = self._object_detector.detect_objects()

        cv2.imshow("Current Feed", frame_with_objects)

    def user_has_quit(self):
        """
        Returns True if the user has quit the program
        :return: bool
        """
        return self._webcam.user_has_quit()

    def release_resources(self):
        """
        Releases the resources used by the webcam
        :return: None
        """
        self._webcam.release_resources()
        cv2.destroyAllWindows()

    def get_goal_location(self):
        """
        Returns the goal location in grid coordinates
        :return: (x, y) tuple
        """
        if self._goal_location is None:
            raise Exception("Goal location not found")

        return self._goal_location


    def get_thymio_location(self):
        """
        Returns the thymio location in grid coordinates
        :return: (x, y) tuple
        """
        if self._thymio_location is None:
            # throw exception:
            raise Exception("Thymio location not found")

        return self._thymio_location

    def _update_thymio_direction(self):
        """
        Updates the thymio direction in grid coordinates
        :return: None
        """
        if self._thymio_corners is None:
            # throw exception:
            raise Exception("Thymio location not found")

        direction = self._thymio_corners[0][0] - self._thymio_corners[0][3]
        normalized_direction = direction / np.linalg.norm(direction)
        self._thymio_direction = normalized_direction

    def get_thymio_direction(self):
        """
        Returns the thymio direction in grid coordinates
        :return: (x, y) tuple
        """
        if self._thymio_corners is None:
            # throw exception:
            raise Exception("Thymio location not found")

        if self._thymio_direction is None:
            self._update_thymio_direction()

        return self._thymio_direction

    def get_grid(self):
        """
        Returns the 2d-grid
        :return: the current grid
        """
        return self._grid

    def get_path(self):
        """
        Returns the path
        :return: the current path
        """
        return self.path


if __name__ == "__main__":
    # Sample usage:
    # width, height = 320, 240
    # should be multiples of 4 and 3 respectively
    width, height = 160, 120
    # width, height = 140, 105
    thymio_marker_id = 4
    goal_marker_id = 5

    # load image from the webcam
    # grid_map = GridMap(width, height, thymio_marker_id, goal_marker_id, load_from_file=None)

    # load image from file
    grid_map = GridMap(width, height, thymio_marker_id, goal_marker_id, load_from_file='images/a1_side_image.png')

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
