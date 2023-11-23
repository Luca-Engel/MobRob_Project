import numpy as np
import cv2

from aruco.marker_recognition import ArUcoMarkerDetector
from opencv.object_recognition import ObjectDetector
from opencv.webcam_input import WebcamFeed
from map.GridMap import GridMap
from map.GridMap import CellType

from queue import PriorityQueue


# from scipy.spatial import distance

class DijkstraNavigation:
    def __init__(self, load_from_file=None):
        self.map = GridMap(load_from_file=load_from_file)
        self.path = []
        # self.start = self.map.get_thymio_grid_coordinates()
        # self.goal = self.map.get_goal_grid_coordinates()
        self.start = self.map.get_thymio_location()
        self.goal = self.map.get_goal_location()

    def check_for_recomputation(self):
        # TODO: check if the start or goal has changed a lot compared to before
        #       if so, recompute the path
        pass

    def get_needed_direction(self):
        # TODO: based on the current location of the thymio and the path, determine the direction it needs to go to
        #       return the direction
        #       Maybe keep track of which parts on the path have been visited already?
        current_thymio_direction = self.map.get_thymio_direction()
        current_thymio_location = self.map.get_thymio_location()
        path = self.map.get_path()

    def compute_dijkstra_path(self):
        grid = self.map.get_grid()
        start = self.start
        goal = self.goal

        rows, cols = grid.shape
        distances = np.full((rows, cols), -1) # Initialize distances with infinity
        predecessors = -1 * np.ones((rows, cols, 2), dtype=int)  # Initialize predecessors with -1
        visited = np.zeros((rows, cols), dtype=bool)

        # Priority queue to keep track of cells to be visited, with priority as the distance
        priority_queue = PriorityQueue()

        # Starting point
        distances[start] = 0
        priority_queue.put((0, start))

        while not priority_queue.empty():
            current_distance, current_cell = priority_queue.get()
            x, y = current_cell

            if visited[y, x]:
                continue

            visited[y, x] = True

            # Check neighbors (up, down, left, right, diagonals)
            neighbors = [(y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1), (y - 1, x - 1), (y - 1, x + 1), (y + 1, x - 1),
                         (y + 1, x + 1)]

            for neighbor_y, neighbor_x in neighbors:
                if 0 <= neighbor_y < rows and 0 <= neighbor_x < cols:
                    cost = 1 if grid[neighbor_y, neighbor_x] in [CellType.FREE, CellType.MARKER,
                                                                 CellType.GOAL] else np.inf
                    if cost != np.inf:
                        if neighbor_y != y and neighbor_x != x:
                            cost = np.sqrt(2)  # diagonal move

                        new_distance = distances[y, x] + cost

                        if new_distance < distances[neighbor_y, neighbor_x] or distances[neighbor_y, neighbor_x] == -1:
                            distances[neighbor_y, neighbor_x] = new_distance
                            predecessors[neighbor_y, neighbor_x] = np.array([y, x])
                            priority_queue.put((new_distance, (neighbor_x, neighbor_y)))

        # Reconstruct path
        path = []
        current = (goal[1], goal[0])
        while not np.array_equal(current, np.array([-1, -1])):
            y, x = current  # divmod(current, cols)
            path.append((x, y))
            current = predecessors[y, x]

        path.reverse()  # Reverse the path to start from the beginning
        self.path = path
        self.map.set_path(path)

        # self.map.set_path([self.start, self.goal])


if __name__ == "__main__":
    dijkstra = DijkstraNavigation(load_from_file='../map/images/a1_side_image.png')

    dijkstra.compute_dijkstra_path()

    print(dijkstra.path)

    while True:
        dijkstra.map.display_grid_as_image()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
