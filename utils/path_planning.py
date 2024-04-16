# Import carla after setting the Python path
import carla
import networkx as nx
import matplotlib.pyplot as plt

def show_graph(graph):
    nx.drwa(graph)
    plt.show()

def path_plan(world, planner):
    wdmap = world.get_map()
    topology = wdmap.get_topology()
    resolution = 3
    new_planner = planner(wdmap, resolution)
    graph = new_planner._graph
    # 2 driving, 3 parking lane
    # reweight the graph


  

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
            


        



def drive(planner, start, dest):
    #driving throught different section of the path
    pass

        
    