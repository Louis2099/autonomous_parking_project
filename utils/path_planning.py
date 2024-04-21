# Import carla after setting the Python path
import carla
import networkx as nx
import matplotlib.pyplot as plt

def show_graph(graph):
    nx.draw(graph)
    plt.show()

def path_cat(path_ids, lanes, planner):
    path = []
    for i in range(len(path_ids)-1):
        s_id = path_ids[i]
        e_id = path_ids[i+1]
        start_wp = [lane[0] for lane in lanes if lane[0].road_id == s_id][0]
        end_wp = [lane[0] for lane in lanes if lane[0].road_id == e_id][0] if i == len(path_ids)-1 else\
                 [lane[1] for lane in lanes if lane[1].road_id == e_id][0]
        path += planner.trace_route(origin = start_wp.transform.location, destination = end_wp.transform.location)
    return path

def target_path(srat_wp, end_wp, planner):
    planner.trace_route(origin = start_wp.transform.location, destination = end_wp.transform.location)

def print_lane(lane_id,topoloby, world):
    lane_start = [lane[0] for lane in topoloby if lane[0].road_id == lane_id][0]
    lane = lane_start.next_until_lane_end(1.0)
    for wp in lane:
        world.debug.draw_string(wp.transform.location, str(wp.road_id), draw_shadow=False,
                                       color=carla.Color(r=0, g=0, b=255), life_time=6000,
                                       persistent_lines=True)
    return lane
    

def path_plan(world, planner):
    """
    return a list with waypoints that can be connected into the path.
    """
    #path_ids = [13, 45, 8, 40, 6, 32, 56]
    path_ids = [35, 6, 30]
    wdmap = world.get_map()
    topology = wdmap.get_topology()
    resolution = 3
    new_planner = planner(wdmap, resolution)
    graph = new_planner._graph
    # 2 driving, 3 parking lane
    """
    print_lane(35, topology, world)
    print_lane(1, topology, world)
    print_lane(6, topology, world)
    quit()
    """
    path = path_cat(path_ids=path_ids, lanes=topology, planner=new_planner)
    for trace in path:
        world.debug.draw_string(trace[0].transform.location, str(trace[0].road_id), draw_shadow=False,
                                       color=carla.Color(r=0, g=0, b=255), life_time=6000,
                                       persistent_lines=True)
        print(trace[1])
    return path
    """
    #search_ask(world, road_ids, lanes=topology)
    show_graph(graph)
    
    print("Before:", len(graph.edges()))
    for e in graph.edges():
        graph[e[0]][e[1]]["weight"] = 0
    width_list = []
    for lane in topology:
        wp_s = lane[0]
        #print(wp_s, type(wp_s), wp_s)
        #print(type(wp_s.lane_width),wp_s.lane_width)
        location = wp_s.transform.location
        road_width = float(wp_s.lane_width)
        road_id = int(wp_s.road_id)
        world.debug.draw_string(location, str(road_id), draw_shadow=False,
                                       color=carla.Color(r=0, g=0, b=255), life_time=6000,
                                       persistent_lines=True)
        if wp_s.lane_type != 2:
            print(int(wp_s.lane_type))
    """

def eu_dist(p1, p2):
    dist = (p1.x - p2.x)^2 + (p1.y - p2.y)^2
        


"""
def search_ask(world, cur_loc, road_ids, lanes):
    road_ids = [14, 53, 18, 0, 10, 2, 16, 11, 7]
    status = [1, 1, 1, 0, 1, 1, 1, 1, 1]
    spots = [lane[0] for lane in lanes if lane[0].road_id in road_ids]
    for spot in spots:
        world.debug.draw_string(spot.transform.location, str(spot.road_id), draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=6000,
                                       persistent_lines=True)
    spots = []
    for i in range(len(road_ids)):
        spots += [lane[0] for lane in lanes if lane[0].road_id == road_ids[i]]
    assert len(spots) == len(road_ids)
    s_id = [i for i in range(len(road_ids)) if eu_dist(spots[i].transform.location, cur_loc) <= 0.5]
    if len(s_id) == 0:
        return None
    else:
        return [(spots[id], status[id]) for id in s_id]
"""
        
    