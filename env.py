import glob
import os
import sys
import queue
import numpy as np
import time
import math
import getpass
import random

user = getpass.getuser() # computer user

# Set the path to the carla folder
if user == "wqiu2":
    path_to_carla = "D:\\carla"
else:
    path_to_carla = os.path.expanduser("~/carla/carla_0_9_15")
print("path_to_carla:", path_to_carla)

# Add the carla library to the Python path
sys.path.append(glob.glob(path_to_carla + '/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
sys.path.append(path_to_carla + "/PythonAPI/carla")

# Import carla after setting the Python path
import carla
from agents.navigation.basic_agent import BasicAgent



def load_custom_map(xodr_path, client):
    """
    Load map from xodr file
    """
    if os.path.exists(xodr_path):
        with open(xodr_path, encoding='utf-8') as od_file:
            try:
                data = od_file.read()
            except OSError:
                print('file could not be readed.')
                sys.exit()
        print('load opendrive map %r.' % os.path.basename(xodr_path))
        vertex_distance = 2.0  # in meters
        max_road_length = 500.0 # in meters
        wall_height = 1.0      # in meters
        extra_width = 0.6      # in meters
        world = client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=True,
                enable_mesh_visibility=True))
    else:
        print(os.getcwd())
        print('file not found.')
    return world

def euclidean_distance(point1, point2):
    """
    Calculate the euclidean distance between two points
    Returns: sqrt((x2-x1)^2 + (y2-y1)^2)
    """
    x1, y1 = point1
    x2, y2 = point2

    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance



class CarlaEnv():
    def __init__(self, port, tm_port, default_map=None, n_vehicles=0, n_walkers = 0):
        self.client = carla.Client("localhost", port)
        self.client.set_timeout(10.0)
        self.world = self.client.load_world(default_map)
        #self.world = load_custom_map(xodr_path, self.client)
        self.actor_list = []
        self.blueprint_library = self.world.get_blueprint_library()
        self.dest = None

        self.traffic_manager = self.client.get_trafficmanager(tm_port)

        self.spawn(n_vehicles, n_walkers)
        self.set_world_settings()
        
    def _get_spawn_point(self):
            # Generate spawn point
            spawn_point = carla.Transform()
            # Get the start point
            start_transform = self.get_start_point()
            start_location = start_transform.location

            # Generate a random location within a certain radius of the start point
            radius = 50  # radius in meters
            angle = np.random.uniform(0, 2*math.pi)  # random angle
            distance = np.random.uniform(0, radius)  # random distance within radius
            loc = carla.Location(x=start_location.x + distance * math.cos(angle),
                                y=start_location.y + distance * math.sin(angle),
                                z=start_location.z)
            spawn_point.location = loc
            return spawn_point
    
    def spawn(self, n_vehicles=0, n_walkers=0):
        """
        Spawn actors in the world
        """
        # --------------
        # Spawn ego/hero vehicle
        # --------------
        ego_vehicle_bp = self.blueprint_library.filter('model3')[0]
        ego_vehicle_bp.set_attribute('role_name', 'hero')
        spawn_point = self.get_start_point()
        self.vehicle = self.world.spawn_actor(ego_vehicle_bp, spawn_point)
        self.actor_list.append(self.vehicle)

        # --------------
        # Spawn vehicles
        # https://github.com/carla-simulator/carla/blob/0.9.15/PythonAPI/examples/generate_traffic.py#L222-L257
        # --------------
        batch = []
        vehicles_list = []
        vehicle_bps = self.blueprint_library.filter("vehicle.*")
        for i in range(n_vehicles):
            vehicle_bp = random.choice(vehicle_bps)
            if vehicle_bp.has_attribute('color'):
                color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
                vehicle_bp.set_attribute('color', color)
            if vehicle_bp.has_attribute('driver_id'):
                driver_id = random.choice(vehicle_bp.get_attribute('driver_id').recommended_values)
                vehicle_bp.set_attribute('driver_id', driver_id)
            vehicle_bp.set_attribute('role_name', 'autopilot')

            spawn_point = self._get_spawn_point()
            # spawn the cars and set their autopilot and light state all together
            batch.append(carla.command.SpawnActor(vehicle_bp, spawn_point)
                .then(carla.command.SetAutopilot(carla.command.FutureActor, True,  
                                                 self.traffic_manager.get_port())))

        for result in self.client.apply_batch_sync(batch, True):
            if result.error:
                print(f"[ERROR] Failed to spawn vehicle: {result.error}")
            else:
                vehicles_list.append(result.actor_id)

        
        # --------------
        # Spawn Walkers
        # https://github.com/carla-simulator/carla/blob/0.9.15/PythonAPI/util/performance_benchmark.py#L277-L330
        # https://carla.readthedocs.io/en/0.9.15/core_actors/#:~:text=vehicle.set_light_state(current_lights)-,Walkers,-carla.Walker%20work
        # --------------
        walkers_list = []

        if n_walkers > 0:
            # (1) Spawn walker actors
            batch = []
            walker_speed = []
            walker_bps = self.blueprint_library.filter("walker.pedestrian.*")
            # Keep looping until we have enough walkers
            for i in range(n_walkers):
                # Randomly choose a walker model
                walker_bp = random.choice(walker_bps)
                # set as not invincible
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                # set the max speed
                if walker_bp.has_attribute('speed'):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    print("Walker has no speed")
                    walker_speed.append(0.0)
                spawn_point = self._get_spawn_point()
                batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
            # Apply the actors
            for result in self.client.apply_batch_sync(batch, True):
                if result.error:
                    print("[ERROR] Failed to spawn walker actor:", result.error)
                else:
                    walkers_list.append({"id": result.actor_id, "speed": walker_speed[i]})
            
            # (2) Spawn walker controllers
            batch = []
            walker_controller_bp = self. world.get_blueprint_library().find('controller.ai.walker')
            for walker in walkers_list:
                batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker["id"]))
            # Apply the controllers
            for i, result in enumerate(self.client.apply_batch_sync(batch, True)):
                if result.error:
                    print("[ERROR] Failed to spawn walker controller:", result.error)
                else:
                    walkers_list[i]["contr_id"] = result.actor_id

            # (3) initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
            # Set how many pedestrians can cross the road
            self.world.set_pedestrians_cross_factor(0.5)

            walker_contr_ids = [w["contr_id"] for w in walkers_list]
            # Flatten the nested list using numpy
            controller_actors = self.world.get_actors(walker_contr_ids)

            # ensures client has received the last transform of the walkers we have just created
            self.world.tick()

            for i, actor in enumerate(controller_actors):
                # start walker
                actor.start()
                # set walk to random point
                actor.go_to_location(self.world.get_random_location_from_navigation())
                # max speed
                actor.set_max_speed(float(walker_speed[i]))

            print(f"Spawned {len(walkers_list)} walkers")



    def set_world_settings(self):
        spectator = self.world.get_spectator()
        transform = self.vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
        carla.Rotation(pitch=-90)))

    def setting_camera(self):
        camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')
        camera.set(FOV=90.0)
        camera.set_image_size(800, 600)
        camera.set_position(x=-3, y=-25, z=10)
        camera.set_rotation(pitch=-90, yaw=0, roll=0)
        self.image_queue = queue.Queue()
        camera.listen(self.image_queue.put)
        self.actor_list.append(camera)  
    
    def draw_path(self, life_time=900.0):
      for point in self.path:
         self.world.debug.draw_string(point, 'O', draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=life_time,
                                       persistent_lines=True)
         
    def get_start_point(self):
        if self.world.get_map().name == 'Carla/Maps/Town05':
            spawn_point = carla.Transform(carla.Location(-38, -30, 0), carla.Rotation(0, 0, -90))
            start_point = self.world.get_map().get_waypoint(spawn_point.location, project_to_road=True)#, lane_type=carla.LaneType.Sidewalk)
            print(start_point.lane_id)
            print(start_point.lane_type)
            start_transform =carla.Transform(carla.Location(start_point.transform.location.x, 
                                                            start_point.transform.location.y, 0.5), 
                                             start_point.transform.rotation)
        return start_transform
        #return spawn_point
    
    def mark_parking_spots(self):
        self.width = 2.8
        self.length = 6
        self.vacancy_matrix = np.asarray([
            [0, 1, 1, 0, 0, 0, 0, 0, 1, 1],
            [0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
            [1, 1, 0, 0, 1, 0, 0, 1, 1, 0],
            [0, 1, 0, 0, 0, 0, 0, 1, 0, 0],
            [1, 1, 0, 0, 0, 0, 0, 0, 1, 1]
            ])
        row_x = [self.path[0].x + 10, 
                 self.path[2].x-3, 
                 self.path[2].x+5, 
                 self.path[4].x-3,
                 self.path[4].x+5,
                 self.path[6].x-5]
        
        col_y = np.arange(-45+self.width/2, -15-self.width/2, self.width)
        location_list = []
        for x in row_x:
            for y in col_y:
                location_list.append(carla.Location(x, y, 0))
        
        for point in location_list:
           self.world.debug.draw_string(point, 'O', draw_shadow=False,
                                         color=carla.Color(r=0, g=255, b=0), life_time=600,
                                         persistent_lines=True)
        
        
        
        
    def path_planning(self):
        self.path = [carla.Location(-38, -30, 0), 
                     carla.Location(-38, -47, 0),
                     carla.Location(-19, -47, 0),
                     carla.Location(-19, -13, 0),
                     carla.Location(-3, -13, 0),
                     carla.Location(-3, -47, 0),
                     carla.Location(16, -47, 0),
                     carla.Location(16, -28, 0)
                     ]
        self.draw_path()
        
    def drive(self):
        #TODO ZEQI
        """
        Enable the vechile to follow the search path
        """
        self.path_planning()
        actor = self.vehicle
        actor_agent = BasicAgent(actor)
        
        for dest in self.path:
            Done = False
            actor_agent.set_destination(dest)
            while not Done:
                actor.apply_control(actor_agent.run_step())
                self.world.wait_for_tick()
                v_loc = actor.get_location()
                dist = euclidean_distance((v_loc.x, v_loc.y), (dest.x, dest.y))
                print(f"DIST {dist}")
                print(f"v_loc {v_loc}")
                print(f"dest {dest}")
                if dist<0.1:
                    Done = True
                    break
    
    def search(self, vl_x, vl_y):
        row_x = [self.path[0].x, self.path[2].x, 0, self.path[4].x, 0, self.path[6].x]
        col_y = [y for y in range(-45, -15, self.width)]
        x = np.argmin(np.asarray(row_x) - vl_x)
        y = np.argmin(np.asarray(col_y) - vl_y)
        if x == 1 or x == 3:
            result = [[x, y, self.vacancy_matrix[x][y]],
                      [x+1, y, self.vacancy_matrix[x+1][y]]]
                
        else:
            result = [[x, y, self.vacancy_matrix[x][y]]]
        
        return result


def spectate(env):
    while(True):
        t = env.world.get_spectator().get_transform()
        #coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
        coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z)
        print (coordinate_str)
        time.sleep(1)
        
def osm_to_xodr(osm_path):
    """
    Convert .osm file to .xodr file
    https://carla.readthedocs.io/en/latest/tuto_G_openstreetmap/
    """
    f = open(osm_path, 'r')
    xodr_path = osm_path.split('.')[0] + '.' + 'xodr'
    osm_data = f.read()
    f.close()
    
    # Define the desired settings. In this case, default values.
    settings = carla.Osm2OdrSettings()
    # Set OSM road types to export to OpenDRIVE
    # NOTE: parking lanes are 'service' roads
    settings.set_osm_way_types(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "service"])
    # Convert to .xodr
    xodr_data = carla.Osm2Odr.convert(osm_data, settings)
    
    # save opendrive file
    f = open(xodr_path, 'w')
    f.write(xodr_data)
    f.close()
    print(f"{xodr_path} created")
    
    
if __name__ == '__main__':
    osm_path = "/home/ubuntu/extreme_driving/jiaxingl/002/maps/p4.osm"
    xodr_path = "/home/ubuntu/extreme_driving/jiaxingl/002/maps/p4.xodr"
    #osm_to_xodr(osm_path)
    #['Town04','Town05']
    default_map = 'Town05'

    if user == "wqiu2":
        port = 2000
    else:
        port = 6000
    tm_port = 2001
    n_walkers = 50 # pedestrians
    n_vehicles = 50
    env = CarlaEnv(port, tm_port, default_map, n_vehicles, n_walkers)

    #spectate(env)
    
    
    env.path_planning()
    env.mark_parking_spots()
    #env.vehicle.set_transform(carla.Transform(carla.Location(-38, -47, 0), carla.Rotation(0, 0, -90)))
    #env.drive()
    
    print("Started simulation. Infinity looping\nCtrl+C to exit")
    while True:
        env.world.tick()
