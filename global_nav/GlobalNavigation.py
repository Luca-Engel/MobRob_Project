import numpy as np
import cv2

from map.GridMap import GridMap
from map.GridMap import CellType

from queue import PriorityQueue

KIDNAP_MIN_DISTANCE = 15


class DijkstraNavigation:
    def __init__(self, load_from_file=None):
        self.map = GridMap(load_from_file=load_from_file)
        self.path = []
        # self.start = self.map.get_thymio_grid_coordinates()
        # self.goal = self.map.get_goal_grid_coordinates()
        self.start = self.map.get_thymio_location()
        self.goal = self.map.get_goal_location()
        self.is_path_up_to_date = False
        self._last_known_thymio_location = self.map.get_thymio_location()
        self._last_known_goal_location = self.map.get_goal_location()

    def recompute_if_necessary(self):
        """
        Checks if the path needs to be recomputed (i.e., if the start or goal has changed a lot compared to before)
        :return: The new path if it was recomputed, None otherwise
        """
        if not self._has_goal_been_kidnapped() and not self._has_thymio_been_kidnapped():
            return None

        self.start = self.map.get_thymio_location()
        self.goal = self.map.get_goal_location()
        self._last_known_goal_location = self.map.get_goal_location()
        self._last_known_thymio_location = self.map.get_thymio_location()

        return self.compute_dijkstra_path()

    def _has_goal_been_kidnapped(self):
        """
        Checks if the goal has been kidnapped
        :return: True if the goal has been kidnapped, False otherwise
        """
        last_goal_location = np.array(self._last_known_goal_location)
        current_goal_location = np.array(self.map.get_goal_location())

        distance = np.linalg.norm(last_goal_location - current_goal_location)

        if distance > 10:
            print("Distance goal: ", distance)
        return distance > KIDNAP_MIN_DISTANCE

    def _has_thymio_been_kidnapped(self):
        """
        Checks if the Thymio has been kidnapped
        :return: True if the Thymio has been kidnapped, False otherwise
        """
        last_thymio_location = np.array(self._last_known_thymio_location)
        current_thymio_location = np.array(self.map.get_thymio_location())

        distance = np.linalg.norm(last_thymio_location - current_thymio_location)
        self._last_known_thymio_location = current_thymio_location
        if distance > 10:
            print("Distance thymio: ", distance)

        return distance > KIDNAP_MIN_DISTANCE

    def get_path_direction(self):
        """
        Returns the direction the path intends for the Thymio to go
        :return: The direction the path intends for the Thymio to go
        """
        # TODO: based on the current location of the thymio and the path, determine the direction it needs to go to
        #       return the direction
        #       Maybe keep track of which parts on the path have been visited already?
        thymio_location = self.map.get_thymio_location()
        thymio_direction = self.map.get_thymio_direction()
        path = self.map.get_path()

    def get_thymio_direction(self):
        """
        Returns the direction the Thymio is facing
        :return: The direction the Thymio is facing
        """
        return self.map.get_thymio_direction()

    def compute_dijkstra_path(self):
        """
        Computes the shortest path from the start to the goal using Dijkstra's algorithm
        :return: The shortest path from the start to the goal
        """
        grid = self.map.get_grid()
        start = self.start
        goal = self.goal

        rows, cols = grid.shape
        distances = np.full((rows, cols), -1)  # Initialize distances with infinity
        predecessors = -1 * np.ones((rows, cols, 2), dtype=int)  # Initialize predecessors with -1
        visited = np.zeros((rows, cols), dtype=bool)

        # Priority queue to keep track of cells to be visited, with priority as the distance
        priority_queue = PriorityQueue()

        # Starting point
        distances[(start[1], start[0])] = 0
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
        self.is_path_up_to_date = True

        return path

    def update_start_location(self, start):
        """
        Updates the start location
        :param start: The new start location
        :return: None
        """
        self.start = start
        self.is_path_up_to_date = False

    def update_goal_location(self, goal):
        """
        Updates the goal location
        :param goal: The new goal location
        :return: None
        """
        self.goal = goal
        self.is_path_up_to_date = False


if __name__ == "__main__":
    # dijkstra = DijkstraNavigation(load_from_file='../map/images/a1_side_image.png')
    # dijkstra = DijkstraNavigation(load_from_file='../map/images/a1_side_obstacles_cut_out.png')
    dijkstra = DijkstraNavigation(load_from_file=None)

    path = dijkstra.compute_dijkstra_path()

    print(path)

    while True:
        dijkstra.map.update_goal_and_thymio_grid_location()
        dijkstra.map.display_grid_as_image()
        dijkstra.map.display_feed()

        dijkstra.recompute_if_necessary()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
