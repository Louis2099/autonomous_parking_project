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



def _vehicle_obstacle_detected(self, vehicle_list=None, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
    """
    Method to check if there is a vehicle in front of the agent blocking its path.

        :param vehicle_list (list of carla.Vehicle): list contatining vehicle objects.
            If None, all vehicle in the scene are used
        :param max_distance: max freespace to check for obstacles.
            If None, the base threshold value is used
    """
    def get_route_polygon():
        route_bb = []
        extent_y = self._vehicle.bounding_box.extent.y
        r_ext = extent_y + self._offset
        l_ext = -extent_y + self._offset
        r_vec = ego_transform.get_right_vector()
        p1 = ego_location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
        p2 = ego_location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
        route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

        for wp, _ in self._local_planner.get_plan():
            if ego_location.distance(wp.transform.location) > max_distance:
                break

            r_vec = wp.transform.get_right_vector()
            p1 = wp.transform.location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
            p2 = wp.transform.location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
            route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

        # Two points don't create a polygon, nothing to check
        if len(route_bb) < 3:
            return None

        return Polygon(route_bb)

    if self._ignore_vehicles:
        return (False, None, -1)

    if not vehicle_list:
        vehicle_list = self._world.get_actors().filter("*vehicle*")

    if not max_distance:
        max_distance = self._base_vehicle_threshold

    ego_transform = self._vehicle.get_transform()
    ego_location = ego_transform.location
    ego_wpt = self._map.get_waypoint(ego_location)

    # Get the right offset
    if ego_wpt.lane_id < 0 and lane_offset != 0:
        lane_offset *= -1

    # Get the transform of the front of the ego
    ego_front_transform = ego_transform
    ego_front_transform.location += carla.Location(
        self._vehicle.bounding_box.extent.x * ego_transform.get_forward_vector())

    opposite_invasion = abs(self._offset) + self._vehicle.bounding_box.extent.y > ego_wpt.lane_width / 2
    use_bbs = self._use_bbs_detection or opposite_invasion or ego_wpt.is_junction

    # Get the route bounding box
    route_polygon = get_route_polygon()

    for target_vehicle in vehicle_list:
        if target_vehicle.id == self._vehicle.id:
            continue

        target_transform = target_vehicle.get_transform()
        if target_transform.location.distance(ego_location) > max_distance:
            continue

        target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)

        # General approach for junctions and vehicles invading other lanes due to the offset
        if (use_bbs or target_wpt.is_junction) and route_polygon:

            target_bb = target_vehicle.bounding_box
            target_vertices = target_bb.get_world_vertices(target_vehicle.get_transform())
            target_list = [[v.x, v.y, v.z] for v in target_vertices]
            target_polygon = Polygon(target_list)

            if route_polygon.intersects(target_polygon):
                return (True, target_vehicle, compute_distance(target_vehicle.get_location(), ego_location))

        # Simplified approach, using only the plan waypoints (similar to TM)
        else:

            if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id  + lane_offset:
                next_wpt = self._local_planner.get_incoming_waypoint_and_direction(steps=3)[0]
                if not next_wpt:
                    continue
                if target_wpt.road_id != next_wpt.road_id or target_wpt.lane_id != next_wpt.lane_id  + lane_offset:
                    continue

            target_forward_vector = target_transform.get_forward_vector()
            target_extent = target_vehicle.bounding_box.extent.x
            target_rear_transform = target_transform
            target_rear_transform.location -= carla.Location(
                x=target_extent * target_forward_vector.x,
                y=target_extent * target_forward_vector.y,
            )

            if is_within_distance(target_rear_transform, ego_front_transform, max_distance, [low_angle_th, up_angle_th]):
                return (True, target_vehicle, compute_distance(target_transform.location, ego_transform.location))

    return (False, None, -1)