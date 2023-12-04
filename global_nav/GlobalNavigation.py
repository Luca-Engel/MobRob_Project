import numpy as np
import cv2
from tdmclient import ClientAsync, aw

from map.GridMap import GridMap
from map.GridMap import CellType
from map.GridMap import CELL_ATTAINED_DISTANCE
from thymio.MotionControl import Motion
from thymio.MotionControl import rotation_nextpoint
from thymio.LocalNavigation import LocalNavigation, LocalNavState, DangerState

from queue import PriorityQueue

KIDNAP_MIN_DISTANCE = 70
CIRCLE_COUNTER_BUFFER = 40


class DijkstraNavigation:
    """
        Class for computing the shortest path from the start to the goal using Dijkstra's algorithm

        Attributes
        ----------
        map : GridMap
            The map of the environment
        path : list
            The shortest path from the start to the goal
        start : tuple
            The start location
        goal : tuple
            The goal location
        is_path_up_to_date : bool
            Whether the path is up to date or not
    """

    def __init__(self, load_from_file=None):
        self.map = GridMap(load_from_file=load_from_file)
        self.path = []
        self.start = self.map.get_kalman_thymio_location()
        self.goal = self.map.get_goal_location()
        self.is_path_up_to_date = False
        self._last_known_thymio_location = self.map.get_kalman_thymio_location()
        self._last_known_goal_location = self.map.get_goal_location()
        self._next_direction_change_idx = 0
        self.direction_changes = []

    def recompute_if_necessary(self, local_nav):
        """
        Checks if the path needs to be recomputed (i.e., if the start or goal has changed a lot compared to before)
        :return: The new path if it was recomputed, None otherwise
        """
        if not self._has_goal_been_kidnapped() and not self._has_thymio_been_kidnapped():
            return None

        return self.handle_kidnap(local_nav)

    def handle_kidnap(self, local_nav):
        """
        Handles the case where the Thymio or the goal has been kidnapped
        :param local_nav: The local navigation instance
        :return: The new path to the goal
        """
        self.map.remove_path_from_grid()

        local_nav.reset_state() # Important, as we might kidnap the thymio as it circles
        self.map.kalman_filter.set_thymio_kidnap_location(self.map.get_camera_thymio_location_est(),
                                                          self.map.get_camera_thymio_direction_est())

        self.start = self.map.get_kalman_thymio_location()
        self.goal = self.map.get_goal_location()
        self._last_known_goal_location = self.map.get_goal_location()
        self._last_known_thymio_location = self.map.get_kalman_thymio_location()
        self._next_direction_change_idx = 0

        path = self.compute_dijkstra_path()
        self.direction_changes = self._find_direction_changes()
        return path

    def _has_goal_been_kidnapped(self):
        """
        Checks if the goal has been kidnapped
        :return: True if the goal has been kidnapped, False otherwise
        """
        last_goal_location = np.array(self._last_known_goal_location)
        current_goal_location = np.array(self.map.get_goal_location())

        distance = np.linalg.norm(last_goal_location - current_goal_location)

        return distance > KIDNAP_MIN_DISTANCE

    def _has_thymio_been_kidnapped(self):
        """
        Checks if the Thymio has been kidnapped
        :return: True if the Thymio has been kidnapped, False otherwise
        """
        last_thymio_location = self._last_known_thymio_location
        current_thymio_location = self.map.get_camera_thymio_location_est()

        if current_thymio_location is None:
            return False

        current_thymio_location = np.array(current_thymio_location)

        if last_thymio_location is None:
            self._last_known_thymio_location = current_thymio_location
            return False

        last_thymio_location = np.array(last_thymio_location)

        distance = np.linalg.norm(last_thymio_location - current_thymio_location)
        self._last_known_thymio_location = current_thymio_location

        return distance > KIDNAP_MIN_DISTANCE

    def _find_direction_changes(self):
        """
        Finds the direction changes in the path
        :return: The direction changes in the path
        """
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

        direction_changes.append((prev_x, prev_y))

        self.map.set_direction_changes(direction_changes)

        self.direction_changes = direction_changes
        return direction_changes

    def get_thymio_and_path_directions(self):
        """
        Returns the normalized thymio direction and the wanted path direction vectors
        :return: (thymio_direction, wanted_path_direction) where thymio_direction is the direction the Thymio is facing and wanted_path_direction is the direction the path intends for the Thymio to go
        """
        thymio_location = np.array(self.map.get_kalman_thymio_location())
        thymio_direction = np.array(self.map.get_kalman_thymio_direction())
        direction_changes = self.map.direction_changes

        if direction_changes is None or self._next_direction_change_idx >= len(direction_changes):
            return (0, 0), (0, 0)

        wanted_path_direction = np.subtract(direction_changes[self._next_direction_change_idx], thymio_location)

        normalized_thymio_direction = thymio_direction / np.linalg.norm(thymio_direction)

        # Prevent division by 0
        if np.linalg.norm(wanted_path_direction) == 0:
            return normalized_thymio_direction, (0, 0)
        normalized_wanted_path_direction = wanted_path_direction / np.linalg.norm(wanted_path_direction)
        return normalized_thymio_direction, normalized_wanted_path_direction

    def get_thymio_direction(self):
        """
        Returns the direction the Thymio is facing
        :return: The direction the Thymio is facing
        """
        return self.map.get_kalman_thymio_direction()

    def compute_dijkstra_path(self):
        """
        Computes the shortest path from the start to the goal using Dijkstra's algorithm
        :return: The shortest path from the start to the goal
        """
        grid = self.map.get_grid()
        self.start = tuple(map(int, self.start))
        start = self.start
        goal = self.goal

        rows, cols = grid.shape
        distances = np.full((rows, cols), -1)
        nb_turns = np.full((rows, cols), -1)
        predecessors = -1 * np.ones((rows, cols, 2), dtype=int)
        visited = np.zeros((rows, cols), dtype=bool)

        priority_queue = PriorityQueue()

        distances[(start[1], start[0])] = 0
        nb_turns[(start[1], start[0])] = 0
        predecessors[(start[1], start[0])] = np.array((start[1], start[0]))
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

                        if new_distance <= distances[neighbor_y, neighbor_x] or distances[neighbor_y, neighbor_x] == -1:
                            dir = np.subtract((neighbor_y, neighbor_x), (y, x))

                            curr_nb_turns = nb_turns[y, x]
                            if not np.array_equal(dir, np.subtract((y, x), predecessors[y, x])):
                                curr_nb_turns += 0.1

                            if curr_nb_turns < nb_turns[neighbor_y, neighbor_x] or nb_turns[
                                neighbor_y, neighbor_x] == -1:
                                distances[neighbor_y, neighbor_x] = new_distance
                                predecessors[neighbor_y, neighbor_x] = np.array([y, x])
                                priority_queue.put((new_distance, (neighbor_x, neighbor_y)))
                                nb_turns[neighbor_y, neighbor_x] = curr_nb_turns

        # Reconstruct path
        path = []
        current = (goal[1], goal[0])
        while not (np.array_equal(current, (start[1], start[0])) or np.array_equal(current, np.array([-1, -1]))):
            y, x = current
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

    def update_navigation(self, local_nav):
        """
        Updates the navigation
        :return: None
        """
        self.map.update_goal_and_thymio_grid_location()
        self.recompute_if_necessary(local_nav)
        self._update_current_path_direction_idx()

    def _update_current_path_direction_idx(self):
        """
        Updates the current path direction index
        :return: None
        """
        thymio_location = np.array(self.map.get_kalman_thymio_location())
        direction_changes = self.map.direction_changes

        if direction_changes is None or self._next_direction_change_idx >= len(direction_changes):
            return

        next_direction_change = direction_changes[self._next_direction_change_idx]
        distance = np.linalg.norm(thymio_location - next_direction_change)

        if distance < CELL_ATTAINED_DISTANCE:
            self._next_direction_change_idx += 1

    def has_thymio_reached_goal(self):
        """
        Checks if the Thymio has reached the goal
        :return: True if the Thymio has reached the goal, False otherwise
        """
        thymio_location = np.array(self.map.get_kalman_thymio_location())
        goal_location = np.array(self.map.get_goal_location())

        distance = np.linalg.norm(thymio_location - goal_location)

        return distance < CELL_ATTAINED_DISTANCE

    def display_grid_as_image(self):
        """
        Displays the grid as an image
        :return: None
        """
        self.map.display_grid_as_image(self._next_direction_change_idx)

    def display_feed(self):
        """
        Displays the feed
        :return: None
        """
        self.map.display_feed()

    def find_closest_cell_on_path(self, thymio_location):
        """
        Finds the closest cell on the path and stores it in the map instance
        :param thymio_location: The Thymio's location
        :return: None The closest cell on the path
        """
        self.map.set_last_known_cell_before_danger(thymio_location)

    def handle_local_navigation_exit(self, thymio_location):
        """
        Handles the exit of local navigation and takeover of global navigation
        :param thymio_location: The Thymio's location
        :return: None
        """
        x_thymio, y_thymio = thymio_location

        self._next_direction_change_idx = 0
        x_dir_change, y_dir_change = self.map.direction_changes[self._next_direction_change_idx]
        for (x_path, y_path) in self.path:
            if x_path == x_dir_change and y_path == y_dir_change:
                self._next_direction_change_idx += 1
                x_dir_change, y_dir_change = self.map.direction_changes[self._next_direction_change_idx]

            if x_path == x_thymio and y_path == y_thymio:
                break


async def main(node):
    """
    Main function
    :param node: The Thymio node instance
    :return: None
    """
    print("initializing")

    aw(node.lock())

    motion_control = Motion(node)

    aw(node.set_variables(motion_control.motors(0, 0)))

    dijkstra = DijkstraNavigation(load_from_file=None)

    path = dijkstra.compute_dijkstra_path()

    dijkstra._find_direction_changes()

    local_nav = LocalNavigation()  # Init LocalNav

    while True:                     # This is the main loop, runs until the goal is reached
        aw(node.wait_for_variables())
        dijkstra.map.update_kalman_filter(speed_left_wheel=node["motor.left.speed"],
                                          speed_right_wheel=node["motor.right.speed"])

        dijkstra.update_navigation(local_nav)

        dijkstra.display_grid_as_image()
        dijkstra.display_feed()

        thymio_direction, wanted_path_direction = dijkstra.get_thymio_and_path_directions()
        thymio_location = dijkstra.map.get_kalman_thymio_location()

        local_nav.update_prox(node["prox.horizontal"])
        danger_level = local_nav.judge_severity()
        if danger_level == DangerState.SAFE and local_nav.state == LocalNavState.START:
            dijkstra.find_closest_cell_on_path(thymio_location)  # Only compute when Global Nav is active

        if danger_level != DangerState.SAFE or local_nav.state != LocalNavState.START: #LocalNav is active
            motor_speeds = node["motor.left.target"], node["motor.right.target"]    #old targets, needed in case of faulty call

            #Add a direction change to the list's head, in case of local navigation while thymio has not moved yet
            local_nav_direction_changes = np.insert(np.array(dijkstra.direction_changes), 0,
                                                    (dijkstra.start[0], dijkstra.start[1]), axis=0)
            local_nav_direction_change_idx = dijkstra._next_direction_change_idx + 1

            motor_speeds = local_nav.run(local_nav_direction_changes, local_nav_direction_change_idx,
                                         motor_speeds)
            aw(node.set_variables(motion_control.motors(int(motor_speeds[0]), int(motor_speeds[1]))))

            resume_path_cell = dijkstra.map.check_if_returned_to_path()
            if (danger_level != DangerState.STOP and
                    (local_nav.circle_counter > CIRCLE_COUNTER_BUFFER and resume_path_cell is not None)):  # Back to path, wtih some slack
                # Done with local nav
                local_nav.reset_state()
                dijkstra.handle_local_navigation_exit(thymio_location=resume_path_cell)

            continue  # disregards the rest of the while loop, which is in charge of Global Nav

        position = 1

        if dijkstra.has_thymio_reached_goal():

            aw(node.set_variables(motion_control.motors(0, 0)))
            while True:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    aw(node.set_variables(motion_control.motors(0, 0)))
                    aw(node.unlock())
                    return


        thymio_angle = rotation_nextpoint(thymio_direction) % 360
        wanted_angle = rotation_nextpoint(wanted_path_direction) % 360
        path = dijkstra.map.get_path()
        dir_change_cell = dijkstra.direction_changes[dijkstra._next_direction_change_idx]
        for i in range(len(path)):
            x, y = path[i]
            x_dir_change, y_dir_change = dir_change_cell
            if x == x_dir_change and y == y_dir_change and i > 0:
                x_prev, y_prev = path[i - 1]
                if x == x_prev and y - 1 == y_prev:
                    break

        change_idx = dijkstra._next_direction_change_idx
        left_speed, right_speed = motion_control.pi_regulation(actual_angle=thymio_angle, wanted_angle=wanted_angle,
                                                               position=position, change_idx=change_idx)

        aw(node.set_variables(motion_control.motors(left_speed, right_speed)))


if __name__ == "__main__":
    client = ClientAsync()
    node = aw(client.wait_for_node())

    try:
        ClientAsync.run_async_program(lambda: main(node))

    except TypeError:
        print("No valid path found")
        cv2.destroyAllWindows()
        aw(node.unlock())
    except Exception as e:
        print("Error:", e)
        cv2.destroyAllWindows()
        aw(node.unlock())
