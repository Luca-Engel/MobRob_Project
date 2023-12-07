import numpy as np
import cv2
from tdmclient import ClientAsync, aw

from map.GridMap import GridMap
from map.GridMap import CellType
from map.GridMap import CELL_ATTAINED_DISTANCE
from thymio.MotionControl import Motion, MOVE
from thymio.MotionControl import rotation_nextpoint
from thymio.LocalNavigation import LocalNavigation, LocalNavState, DangerState
from global_nav.GlobalNavigation import DijkstraNavigation

CIRCLE_COUNTER_BUFFER = 40

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

        if danger_level != DangerState.SAFE or local_nav.state != LocalNavState.START:  # LocalNav is active
            motor_speeds = node["motor.left.target"], node["motor.right.target"]  # old targets, needed in case of faulty call

            # Add a direction change to the list's head, in case of local navigation while thymio has not moved yet
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

        movement = MOVE

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
        left_speed, right_speed = motion_control.motion_regulation(actual_angle=thymio_angle, wanted_angle=wanted_angle,
                                                                   movement=movement, change_idx=change_idx)

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
