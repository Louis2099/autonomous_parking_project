# Copied from https://github.com/carla-simulator/carla/blob/0.9.15/PythonAPI/carla/agents/navigation/behavior_agent.py#L131-L166
def collision_and_car_avoid_manager(self, waypoint):
    """
    This module is in charge of warning in case of a collision
    and managing possible tailgating chances.

        :param location: current location of the agent
        :param waypoint: current waypoint of the agent
        :return vehicle_state: True if there is a vehicle nearby, False if not
        :return vehicle: nearby vehicle
        :return distance: distance to nearby vehicle
    """

    vehicle_list = self._world.get_actors().filter("*vehicle*")

    def dist(v):
        return v.get_location().distance(waypoint.transform.location)

    vehicle_list = [
        v for v in vehicle_list if dist(v) < 45 and v.id != self._vehicle.id
    ]

    if self._direction == RoadOption.CHANGELANELEFT:
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
            up_angle_th=180,
            lane_offset=-1,
        )
    elif self._direction == RoadOption.CHANGELANERIGHT:
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
            up_angle_th=180,
            lane_offset=1,
        )
    else:
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 3),
            up_angle_th=30,
        )

        # Check for tailgating
        if (
            not vehicle_state
            and self._direction == RoadOption.LANEFOLLOW
            and not waypoint.is_junction
            and self._speed > 10
            and self._behavior.tailgate_counter == 0
        ):
            self._tailgating(waypoint, vehicle_list)

    return vehicle_state, vehicle, distance


def pedestrian_avoid_manager(self, waypoint):
    """
    This module is in charge of warning in case of a collision
    with any pedestrian.

        :param location: current location of the agent
        :param waypoint: current waypoint of the agent
        :return vehicle_state: True if there is a walker nearby, False if not
        :return vehicle: nearby walker
        :return distance: distance to nearby walker
    """

    walker_list = self._world.get_actors().filter("*walker.pedestrian*")

    def dist(w):
        return w.get_location().distance(waypoint.transform.location)

    walker_list = [w for w in walker_list if dist(w) < 10]

    if self._direction == RoadOption.CHANGELANELEFT:
        walker_state, walker, distance = self._vehicle_obstacle_detected(
            walker_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
            up_angle_th=90,
            lane_offset=-1,
        )
    elif self._direction == RoadOption.CHANGELANERIGHT:
        walker_state, walker, distance = self._vehicle_obstacle_detected(
            walker_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 2),
            up_angle_th=90,
            lane_offset=1,
        )
    else:
        walker_state, walker, distance = self._vehicle_obstacle_detected(
            walker_list,
            max(self._behavior.min_proximity_threshold, self._speed_limit / 3),
            up_angle_th=60,
        )

    return walker_state, walker, distance
