# Import carla after setting the Python path
import carla
import networkx as nx
import matplotlib.pyplot as plt
import math
import sys
sys.path.append('utils/')
from graph import *
import numpy as np

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
    

def path_plan(world, planner, fixed = False, start = 25, exit = 28):
    """
    entrance lane 10: 36
    entrance lane 43: 37
    exit lane 8
    return a list with waypoints that can be connected into the path.
    """
    wdmap = world.get_map()
    topology = wdmap.get_topology()
    resolution = 3
    new_planner = planner(wdmap, resolution)
    graph = new_planner._graph
    if fixed == True:
        #path_ids = [13, 45, 8, 40, 6, 32, 56]
        #path_ids = [35, 6, 30]
        path_ids = [16, 49, 6]
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
        
    else:
        weighted_edge = [4, 5, 7, 10]
        start = start          # start of the parking lot (should be edge id)
        exit = exit           # exit of the parking lot (should be edge id)
        #search_ask(world, road_ids, lanes=topology)
        #show_graph(graph)
        """
        for node, attrs in graph.nodes(data=True):
            print("Node {}: {}".format(node, attrs))
        for u, v, attrs in graph.edges(data=True):
            print("Edge {} {}: {}".format(u, v, int(attrs['entry_waypoint'].road_id)))
        """
        for u, v, attrs in graph.edges(data=True):
            if  int(attrs['entry_waypoint'].road_id) in weighted_edge:
                graph[u][v]["gain"] = 10
                graph[u][v]["cost"] = 1
            else:
                graph[u][v]["gain"] = 0
                graph[u][v]["cost"] = 1
        path = optimize_path(graph, start, exit)      # list of edge ids                        
    return path

def show_road_id(world, topology):
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


def eu_dist(p1, p2):
    dist = math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y),2))
    return dist
        



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

def example_search(spot_loc_mx, v_mx, cur_loc):
    serach_radius = 6
    dist_mx = spot_loc_mx - np.asarray([cur_loc.x, cur_loc.y, cur_loc.z])
    dist = np.linalg.norm(dist_mx, axis=2)
    
    assert dist.shape == spot_loc_mx.shape[:2]
    s_id = np.where(dist <= serach_radius)
    # expect where reutn a list of index
    # if len(s_id[0]) != 0:
    #     print(s_id)
    #     print(type(s_id), len(s_id))
    if len(s_id[0]) ==0:
        return [0,0,0]
    
    for i in range(len(s_id[0])):
        if v_mx[s_id[0][i]][s_id[1][i]] == 0:
            yaw = [90] if s_id[1][i] % 2 == 0 else [270]
            return list(spot_loc_mx[s_id[0][i]][s_id[1][i]]) + yaw
    return [0,0,0]