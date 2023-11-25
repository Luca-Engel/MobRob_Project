import numpy as np
import cv2

from map.GridMap import GridMap
from map.GridMap import CellType

from queue import PriorityQueue

# MIN NORM DISTANCE TO CELL TO BE CONSIDERED AS ATTAINED
CELL_ATTAINED_DISTANCE = 4

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
        self._next_direction_change_idx = 0

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
        self._next_direction_change_idx = 0

        path = self.compute_dijkstra_path()
        find_direction_changes = self._find_direction_changes()
        # self.map.set_direction_changes(direction_changes)
        return path

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


    def _find_direction_changes(self):
        if len(self.path) < 2:
            return []

        direction_changes = []
        old_dx, old_dy = np.subtract(self.path[1], self.path[0])

        prev_x, prev_y = self.path[0]

        for x, y in self.path[1:]:
            dx = x - prev_x
            dy = y - prev_y

            if dx != old_dx or dy != old_dy:
                # Diagonal movement
                direction_changes.append((prev_x, prev_y))
                old_dx, old_dy = dx, dy

            prev_x, prev_y = x, y

        self.map.set_direction_changes(direction_changes)
        return direction_changes


    def get_thymio_and_path_directions(self):
        """
        Returns the normalized thymio direction and the wanted path direction vectors
        :return: (thymio_direction, wanted_path_direction) where thymio_direction is the direction the Thymio is facing and wanted_path_direction is the direction the path intends for the Thymio to go
        """
        thymio_location = np.array(self.map.get_thymio_location())
        thymio_direction = np.array(self.map.get_thymio_direction())
        path = self.map.get_path()
        direction_changes = self.map.direction_changes

        if (direction_changes is None or self._next_direction_change_idx >= len(direction_changes)):
            return (0, 0), (0, 0)

        wanted_path_direction = np.subtract(direction_changes[self._next_direction_change_idx], thymio_location)

        # draw arrowed line for wanted path direction and actual thymio direction
        img = cv2.arrowedLine(self.map.grid_image, tuple(thymio_location), tuple(np.add(thymio_location, 10*wanted_path_direction)), (0, 0, 0), 2)
        img = cv2.arrowedLine(img, tuple(thymio_location), tuple(np.add(thymio_location, tuple(map(int, thymio_direction)))), (0, 255, 0), 2)
        cv2.imshow("direction", img)

        normalized_thymio_direction = thymio_direction / np.linalg.norm(thymio_direction)
        normalized_wanted_path_direction = wanted_path_direction / np.linalg.norm(wanted_path_direction)
        return normalized_thymio_direction, normalized_wanted_path_direction



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

    def update_navigation(self):
        """
        Updates the navigation
        :return: None
        """

        dijkstra.map.update_goal_and_thymio_grid_location()
        dijkstra.recompute_if_necessary()
        self._update_current_path_direction_idx()


    def _update_current_path_direction_idx(self):
        """
        Updates the current path direction index
        :return: None
        """
        thymio_location = np.array(self.map.get_thymio_location())
        direction_changes = self.map.direction_changes

        if direction_changes is None or self._next_direction_change_idx >= len(direction_changes):
            return

        next_direction_change = direction_changes[self._next_direction_change_idx]
        distance = np.linalg.norm(thymio_location - next_direction_change)

        if distance < CELL_ATTAINED_DISTANCE:
            print("Reached direction change: ", next_direction_change)
            self._next_direction_change_idx += 1


    def has_thymio_reached_goal(self):
        """
        Checks if the Thymio has reached the goal
        :return: True if the Thymio has reached the goal, False otherwise
        """
        thymio_location = np.array(self.map.get_thymio_location())
        goal_location = np.array(self.map.get_goal_location())

        distance = np.linalg.norm(thymio_location - goal_location)

        return distance < CELL_ATTAINED_DISTANCE


    def display_grid_as_image(self):
        """
        Displays the grid as an image
        :return: None
        """
        self.map.display_grid_as_image()

    def display_feed(self):
        """
        Displays the feed
        :return: None
        """
        self.map.display_feed()

if __name__ == "__main__":
    # dijkstra = DijkstraNavigation(load_from_file='../map/images/a1_side_image.png')
    # dijkstra = DijkstraNavigation(load_from_file='../map/images/a1_side_obstacles_cut_out.png')
    dijkstra = DijkstraNavigation(load_from_file=None)

    path = dijkstra.compute_dijkstra_path()

    print(path)

    direction_changes = dijkstra._find_direction_changes()

    print(direction_changes)

    while True:
        dijkstra.update_navigation()

        dijkstra.display_grid_as_image()
        dijkstra.display_feed()

        thymio_direction, wanted_path_direction = dijkstra.get_thymio_and_path_directions()

        if (dijkstra.has_thymio_reached_goal()):
            print("Reached goal!")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
