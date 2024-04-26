import glob
import os
import sys
import queue
import numpy as np
import time
import math
import getpass
import random
import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator 
import Hybrid_A_Star
from utils.path_planning import *

user = getpass.getuser() # computer user

# Set the path to the carla folder

print("User:", user)
if user == "wqiu2":
    path_to_carla = "D:\\carla"
elif user == "jiaze":
    # path_to_carla = "H:\\carla\WindowsNoEditor"
    path_to_carla = "H:\CARLA\carla"
else:
    path_to_carla = os.path.expanduser("~/carla/carla_0_9_15")
print("path_to_carla:", path_to_carla)

# Add the carla library to the Python path
sys.path.append(glob.glob(path_to_carla + '/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
sys.path.append(path_to_carla + "/PythonAPI/carla")

try: 
    # Add the carla library to the Python path
    sys.path.append(glob.glob(path_to_carla + '/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    sys.path.append(path_to_carla + "/PythonAPI/carla")
except IndexError:
    pass

# Import carla after setting the Python path
import carla
#from agents.navigation.basic_agent import BasicAgent
sys.path.append("H:/CARLA/WindowsNoEditor" + "/PythonAPI/carla")
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner as planner



def load_custom_map(xodr_path, fbx_path, client):
    """
    Load map from xodr file
    """
    print("Load custom map: ", xodr_path)
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
        wall_height = 0      # in meters
        extra_width = 0.6      # in meters
        world = client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=True,
                enable_mesh_visibility=True))
        

        # Spawn parking lot
        import xml.etree.ElementTree as ET
        from pyproj import Proj, transform as pyproj_transform

        # Parse the XODR file
        tree = ET.parse(xodr_path)
        root = tree.getroot()

        # Extract geographic reference information
        header = root.find('header')
        geo_ref = "+proj=tmerc +lat_0=30.161615 +lon_0=-85.59196 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs" # header.find('geoReference').text

        # Extract boundary coordinates for the coordinate transformation
        north = float(header.attrib['north'])
        south = float(header.attrib['south'])
        east = float(header.attrib['east'])
        west = float(header.attrib['west'])

        # Define the projection function for the transformation
        in_proj = Proj(geo_ref)  # Use the geographic reference information from the XODR header
        out_proj = Proj(init='epsg:4326')  # WGS84 coordinate system

        # Define a function to convert local coordinates (s, t) to global coordinates (x, y)
        def local_to_global(s, t):
            x, y = pyproj_transform(in_proj, out_proj, s, t)
            return x, y

        # Find all object elements representing parking spaces
        parking_spaces = root.findall(".//object[@type='parkingSpace']")

        # Extract information for each parking space
        parking_bp = world.get_blueprint_library().find('vehicle.tesla.model3')  # Example blueprint
        for parking_space in parking_spaces:
            id = parking_space.attrib['id']
            s = float(parking_space.attrib['s'])
            t = float(parking_space.attrib['t'])
            hdg = float(parking_space.attrib['hdg'])
            zOffset = float(parking_space.attrib['zOffset'])
            width = float(parking_space.attrib['width'])
            length = float(parking_space.attrib['length'])
            orientation = parking_space.attrib['orientation']

            x, y = local_to_global(s, t)
            # print("x, y:", x, y)
            transform = carla.Transform(carla.Location(north+float(s), east+float(t), 5))  # Adjust the z coordinate as needed
            world.try_spawn_actor(parking_bp, transform)

    else:
        print(os.getcwd())
        print('file not found. could not load custom map!')
        exit()

    # if os.path.exists(fbx_path):
    #     print("Loading fbx file:", fbx_path)
    #     with open(fbx_path, "rb") as f:
    #         fbx_bp = carla.ActorBlueprint(f.read())
    #         fbx_actor = world.spawn_actor(fbx_bp, carla.Transform())

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
        print("Maps:", self.client.get_available_maps())

        self.world = self.client.load_world(default_map)
        # xodr_path = r"D:\autonomous_parking_project\maps\parkinglot03.xodr"
        # fbx_path = None # r"D:\autonomous_parking_project\maps\parkinglot.fbx"
        # geojson_path = r"D:\autonomous_parking_project\maps\xparkinglot.geojson"
        # self.world = load_custom_map(xodr_path, fbx_path, self.client)

        self.actor_list = []
        self.blueprint_library = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.dest = None
        
        self.obstacle_detected = False
        self.location_list = []
        self.traffic_manager = self.client.get_trafficmanager(tm_port)

        self.spawn(n_vehicles, n_walkers)
        self.set_world_settings()
        self.init_sensors()
        self.path_planning(custom_map=True)


        
    def _get_spawn_point(self):
        # Parking lot dimensions
        min_x = -40
        max_x = 18
        min_y = -50
        max_y = -15
        # Generate spawn point
        spawn_point = carla.Transform()
        # Get the start point
        # start_transform = self.get_start_point()
        # start_location = start_transform.location

        # Generate a random location within the parking lot
        loc = carla.Location(x=np.random.uniform(min_x, max_x),
                            y=np.random.uniform(min_y, max_y),
                            # z=start_location.z)
                            z=0.5)
        spawn_point.location = loc
        # Set rotation to be north to south
        spawn_point.rotation.yaw = 0
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
        spawn_point = self.spawn_points[0]
        spawn_point.location.z = 0.1 # Lower it to prevent spawn, bounce, and flipping over!
        
        while True:
            try:
                self.vehicle = self.world.spawn_actor(ego_vehicle_bp, spawn_point)
                print("Spawned ego vehicle at", spawn_point.location)
                break
            except Exception as collision:
                print("Failed to spawn vehicle:", collision)
                self.world.debug.draw_string(spawn_point.location, 'A', draw_shadow=False,
                                       color=carla.Color(r=0, g=0, b=255), life_time=600,
                                       persistent_lines=True)
                spawn_point.location.x += 1
                
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
            if vehicle_bp.has_attribute('speed'):
                vehicle_bp.set_attribute('speed', 3)
            vehicle_bp.set_attribute('role_name', 'autopilot')

            spawn_point = np.random.choice(self.spawn_points)
            spawn_point.location.z = 0.1 # Lower it to prevent spawn, bounce, and flipping over!
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
                spawn_point = np.random.choice(self.spawn_points)
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
                # actor.go_to_location(self.world.get_random_location_from_navigation())
                actor.go_to_location(np.random.choice(self.spawn_points).location)
                # max speed
                actor.set_max_speed(float(walker_speed[i]))

            print(f"Spawned {len(walkers_list)} walkers")



    def set_world_settings(self):
        # Set starting location of the spectator camera
        spectator = self.world.get_spectator()
        transform = self.vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(x=0,y=0, z=50),
        carla.Rotation(pitch=-90)))

        # Set weather of the world
        weather = carla.WeatherParameters(
            cloudiness=0.0,
            precipitation=0.0,
            sun_altitude_angle=10.0,
            sun_azimuth_angle = 70.0,
            precipitation_deposits = 0.0,
            wind_intensity = 0.0,
            fog_density = 0.0,
            wetness = 0.0, 
        )
        self.world.set_weather(weather)

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
         self.world.debug.draw_string(point.location, 'O', draw_shadow=False,
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
        else:
            start_point = self.spawn_points[0]
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
        row_x = [self.ckp[0].x + 10, 
                 self.ckp[2].x-3, 
                 self.ckp[2].x+5, 
                 self.ckp[4].x-3,
                 self.ckp[4].x+5,
                 self.ckp[6].x-5]
        

        col_y = np.arange(-45+self.width/2, -15-self.width/2, self.width)
        self.row_x = row_x
        self.col_y = col_y
        """
        while True:
            rand_x = random.randint(0, len(row_x)-1)
            rand_y = random.randint(0, len(col_y)-1)
            if self.vacancy_matrix[rand_x][rand_y]  == 1:
                continue
            else:
                self.target_spont = [rand_x, rand_y]
                break
        """
        # for testing
        self.target_spont = [1, 0]
        location_list = []
        for id_x in range(len(row_x)):
            for id_y in range(len(col_y)):
                location = carla.Location(row_x[id_x], col_y[id_y], 0)
                if id_x == self.target_spont[0] and id_y == self.target_spont[1]:
                    self.vacancy_matrix[id_x][id_y] = 0
                    self.world.debug.draw_string(location, 'O', draw_shadow=False,
                                         color=carla.Color(r=0, g=255, b=0), life_time=600,
                                         persistent_lines=True)
                else:
                    self.vacancy_matrix[id_x][id_y] = 1
                    self.world.debug.draw_string(location, 'O', draw_shadow=False,
                                            color=carla.Color(r=255, g=0, b=0), life_time=600,
                                            persistent_lines=True)
                location_list.append(location)

        self.location_list = location_list
        
        
        
    def path_planning(self, custom_map=True):
        if custom_map != True:
            self.ckp = [carla.Location(-38, -30, 0), 
                        carla.Location(-38, -47, 0),
                        carla.Location(-19, -47, 0),
                        carla.Location(-19, -13, 0),
                        carla.Location(-3, -13, 0),
                        carla.Location(-3, -47, 0),
                        carla.Location(16, -47, 0),
                        carla.Location(16, -28, 0)
                        ]
            self.path = []
            step_size = 0.1
            for ckp in self.ckp[:-1]:
                delta_x = ckp.x - self.ckp[self.ckp.index(ckp)+1].x
                delta_y = ckp.y - self.ckp[self.ckp.index(ckp)+1].y
                if delta_x != 0:
                    yaw = 0
                    for i in range(int(abs(delta_x)/step_size)):
                        self.path.append(carla.Transform(carla.Location(ckp.x + i*step_size, ckp.y, 0.5), carla.Rotation(0, yaw, 0)))
                else:
                    if delta_y < 0:
                        yaw = 90
                        for i in range(int(abs(delta_y)/step_size)):
                            self.path.append(carla.Transform(carla.Location(ckp.x, ckp.y  + i*step_size, 0.5), carla.Rotation(0, yaw, 0)))
                    else:
                        yaw = -90
                        for i in range(int(abs(delta_y)/step_size)):
                            self.path.append(carla.Transform(carla.Location(ckp.x, ckp.y - i*step_size, 0.5), carla.Rotation(0, yaw, 0)))
        else:
            #show_road_id(self.world, self.world.get_map().get_topology())
            #print_lane(10, self.world.get_map().get_topology(), self.world)
            #print_lane(8, self.world.get_map().get_topology(), self.world)
            self.ckp_wp = path_plan(self.world, planner) # series of waypoints
            self.ckp = [wp.transform.location for wp in self.ckp_wp]
            self.path = []
            rd_id = [ckp.road_id for ckp in self.ckp_wp]
            u_rd_id = []
            for id in rd_id:
                if id not in u_rd_id:
                    u_rd_id.append(id)
            idx = []
            for id in u_rd_id:
                idx.append(rd_id.index(id))
            u_ckp_wp = [self.ckp_wp[id] for id in idx]
            for wp in u_ckp_wp:
                self.path += wp.next_until_lane_end(0.1)
            self.path = [wp.transform for wp in self.path]
        self.draw_path()
    
    

    
    def search(self, vl_x, vl_y):
        #row_x = [self.ckp[0].x, self.ckp[2].x, 0, self.ckp[4].x, 0, self.ckp[6].x]
        #col_y = [y for y in range(-45, -15, self.width)]
        #col_y = np.arange(-45+self.width/2, -15-self.width/2, self.width)
        row_x = self.row_x
        col_y = self.col_y
        x = np.argmin(np.abs(np.asarray(row_x) - vl_x))
        y = np.argmin(np.abs(col_y - vl_y))
        if x == 1 or x == 3:
            result = [[row_x[x], col_y[y], self.vacancy_matrix[x][y], 0],
                      [row_x[x+1], col_y[y], self.vacancy_matrix[x+1][y], 180]]
                
        else:
            result = [[row_x[x], col_y[y], self.vacancy_matrix[x][y], 0],
                      None]

        return result
    
    def init_sensors(self):
        """
        Get the perception of the environment
        https://medium.com/@parthanvelanjeri/a-dip-into-carla-iii-camera-and-other-sensors-484fa039063c
        """
        img_width = 640
        img_height = 480
        self.sensor_data = {"rgb_img": np.zeros((img_width, img_height, 4)),
                            'depth_img': np.zeros((img_width, img_height, 4)),
                            "yolo_results": []}
        conf_threshold = 50 # confidence threshold

        model = YOLO("yolov8n.pt")  # pretrained YOLOv8n model

        # Camera and lidar generation based on this: https://github.com/mjxu96/carlaviz/blob/0.9.15/examples/example.py#L63-L87
        # Attach a camera and a lidar to the ego vehicle
        # RGB Camera
        def rgb_callback(image):
            rgba_img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) #Reshaping with alpha channel
            rgba_img[:,:,3] = 255 #Setting the alpha to 255 

            # Run batched inference on a list of images
            rgb_img = rgba_img[:, :, :3]  # only want 3 channels
            results = model(rgb_img)  # only want 3 channels

            yolo_results = []
            # Process results list
            # https://stackoverflow.com/a/75332799
            for r in results:
                annotator = Annotator(np.ascontiguousarray(rgb_img))
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls # class index
                    obj = model.names[int(c)]
                    conf = box.conf.item()*100 # confidence
                    if conf < conf_threshold:
                        continue # skip if confidence is too low

                    if obj in ["car", "truck", "bus"] :
                        color = (0, 100, 0) # green
                    elif obj in ["person", "dog", "cat"]:
                        color = (0, 0, 255) # red
                    elif obj in ["bicycle", "motorcycle"]:
                        color = (255, 0, 0) # blue
                    else:
                        color = (128, 128, 128) # grey
                    annotator.box_label(b, obj +" "+ str(round(conf))+"%",
                                        color=color)
                    
                    yolo_results.append({"obj": obj, "conf": conf, "box": b})

            img_with_boxes = annotator.result()  
            self.sensor_data['rgb_img'] = img_with_boxes
            self.sensor_data['yolo_results'] = yolo_results


        blueprint_camera = self.blueprint_library.find("sensor.camera.rgb")
        blueprint_camera.set_attribute("image_size_x", str(img_width))  
        blueprint_camera.set_attribute("image_size_y", str(img_height))
        blueprint_camera.set_attribute("fov", "110")
        blueprint_camera.set_attribute("sensor_tick", "0.1")

        # Position relative to vehicle's center
        # x is front/back, y is left/right, z is up/down
        transform_camera = carla.Transform(carla.Location(x=1.0, y=+0.0, z=1.5)) 
        
        camera = self.world.spawn_actor(
            blueprint_camera, transform_camera, attach_to=self.vehicle
        )
        camera.listen(lambda data: rgb_callback(data))
        self.actor_list.append(camera)

        # Depth camera
        def depth_callback(image):
            image.convert(carla.ColorConverter.LogarithmicDepth) # Make it logarithmic depth (gray scale)
            image = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

            annotator = Annotator(np.ascontiguousarray(image))
            
            desired_objects = ["car", "truck", "bus", "person", "dog", "cat", "bicycle", "motorcycle"]

            obj_detected = False
            for r in self.sensor_data['yolo_results']:
                box = r['box']  # get box coordinates in (left, top, right, bottom) format
                obj = r['obj']
                conf = r['conf']

                if obj not in desired_objects:
                    continue # skip if object is not desired

                # Only consider boxes that are within the middle third of the image
                if box[0] < img_width//3 or box[2] > img_width*2//3:
                    continue # skip if box is not in the middle third

                (left, top, right, bottom) = (int(box[0]), int(box[1]), int(box[2]), int(box[3]))

                # Extract the area within the bounding box
                bbox_area = image[top:bottom, left:right]

                """
                https://carla.readthedocs.io/en/0.9.15/ref_sensors/
                The image codifies depth value per pixel using 3 channels of the RGB color space, from less to more significant bytes: R -> G -> B. The actual distance in meters can be decoded with:

                normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
                in_meters = 1000 * normalized
                """
                R = bbox_area[:, :, 0]
                G = bbox_area[:, :, 1]
                B = bbox_area[:, :, 2]
                normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
                depth_in_cm = 1000 * normalized * 100 # in cm
                depth = np.mean(depth_in_cm)  # average depth in the bounding box

                # Larger depth means further away
                if depth <= 50:
                    color = (0, 0, 255) # red
                    obj_detected = True
                else:
                    color = (0, 100, 0) # green

                annotator.box_label(box, str(int(depth)), color=color)

            self.obstacle_detected = obj_detected
            
                
            img_with_boxes = annotator.result()  
            self.sensor_data['depth_img'] = img_with_boxes

        blueprint_camera = self.blueprint_library.find("sensor.camera.depth")
        blueprint_camera.set_attribute("image_size_x", str(img_width))  
        blueprint_camera.set_attribute("image_size_y", str(img_height))
        blueprint_camera.set_attribute("fov", "110")
        blueprint_camera.set_attribute("sensor_tick", "0.1")
        # Position relative to vehicle's center
        # x is front/back, y is left/right, z is up/down
        camera = self.world.spawn_actor(
            blueprint_camera, transform_camera, attach_to=self.vehicle
        )
        camera.listen(lambda data: depth_callback(data))
        self.actor_list.append(camera)

        print("Sensors initialized")

    def park(self, start, goal, ox, oy):
        
        
        path = Hybrid_A_Star.hybrid_a_star_planning(start, goal, ox, oy, Hybrid_A_Star.XY_GRID_RESOLUTION, Hybrid_A_Star.YAW_GRID_RESOLUTION)

        x = path.x_list
        y = path.y_list
        yaw = path.yaw_list

        cpath = []
        ppath = []
        #print(len(x), len(y))
        #agent = BasicAgent(self.vehicle)
        for i in range(len(x)):
            cpath.append(carla.Location(x[i], y[i], 0))
            
        for i in range(len(cpath)):
            #print(yaw[i])
            rotation = carla.Rotation(0, math.degrees(yaw[i]),0)
            p = carla.Transform(cpath[i], rotation)
            ppath.append(p)

        for dest in cpath:
            env.world.debug.draw_string(dest, 'o', draw_shadow=False,
                                         color=carla.Color(r=0, g=0, b=255), life_time=900,
                                         persistent_lines=True)

        
            
        return ppath
        
        # for dest in cpath:
        #     # Done = False
        #     # agent.set_destination(dest)
        #     # while not Done:
        #     #     self.vehicle.apply_control(agent.run_step())
                
        #     #     v_loc = self.vehicle.get_location()
        #     #     dist = euclidean_distance((v_loc.x, v_loc.y), (dest.x, dest.y))
                
        #     #     if dist<0.1:
        #     #         Done = True
        #     #         break
        #     self.vehicle.set_location(dest)
        #     time.sleep(0.001)



        #     #env.vehicle.set_location(dest)

        cpath = []
        
        agent = BasicAgent(self.vehicle)
        for i in range(x):
            cpath[i] = carla.Location(x[i], y[i], 0)
        
        for dest in cpath:
            Done = False
            agent.set_destination(dest)
            while not Done:
                self.vehicle.apply_control(agent.run_step())
                
                v_loc = self.vehicle.get_location()
                dist = euclidean_distance((v_loc.x, v_loc.y), (dest.x, dest.y))
                
                if dist<0.1:
                    Done = True
                    break

    """
    def drive(self):
        #TODO ZEQI
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
    """
    
            

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

    random.seed(0)
    np.random.seed(0)
    osm_path = "/home/ubuntu/extreme_driving/jiaxingl/002/maps/p4.osm"
    xodr_path = "/home/ubuntu/extreme_driving/jiaxingl/002/maps/p4.xodr"
    #osm_to_xodr(osm_path)
    #['Town04','Town05']
    #default_map = 'Town05'
    default_map = 'parkinglotGG_bake'
    
    if user == "wqiu2":
        port = 2000
    else:
        port = 2000
    n_walkers = 20 # pedestrians
    n_vehicles = 20
    tm_port = 2000


    # n_vehicles = 10
    env = CarlaEnv(port, tm_port, default_map, n_vehicles, n_walkers)
    
    #spectate(env)
    
    
    
    #env.mark_parking_spots()
    env.world.debug.draw_string(carla.Location(x=-28.40, y=-12.20, z=0), 'P', draw_shadow=False,
                                       color=carla.Color(r=0, g=0, b=255), life_time=600,
                                       persistent_lines=True)
    # env.drive(env.path)
    #env.vehicle.set_transform(carla.Transform(carla.Location(-38, -47, 0), carla.Rotation(0, 0, -90)))
    #env.drive()
    
    print("Started simulation. Infinite looping\nCtrl+C to exit")

    path_index = 0 # Which point vehicle is at in the path
    flag = False

    
    
    while True:
        env.world.tick()
        env.world.wait_for_tick()

        # Output camera display onto an OpenCV Window
        cv2.imshow("RGB Camera (press q to exit)", env.sensor_data['rgb_img'])
        cv2.imshow("Depth Camera (press q to exit)", env.sensor_data['depth_img'])
        # Exit with q or ctrl+c
        if cv2.waitKey(1) == ord('q'):
            break

        # Drive the vehicle along the path
        if path_index < len(env.path):
            if env.obstacle_detected:
                print("Obstacle detected")
                continue # skip this iteration if obstacle detected

            point = env.path[path_index]
            Done = False

            # search for parking spot
            # if point.rotation.yaw != 0:
            #     result = env.search(point.location.x, point.location.y)
            #     for spot in result:
            #         if spot != None:
            #             if spot[2] == 0:
            #                 print("Parking spot found")
            #                 print(result)
            #                 Done = True
            #                 break

            # example search in custom map
            dest = example_search(dest = carla.Location(x=-29.90, y=-12.20, z=0), cur_loc = point.location)

            if dest != None:
                Done = True

            
            if not Done:
                try:
                    env.vehicle.set_transform(point)
                    path_index +=1
                       

                    # time.sleep(0.001)
                except Exception as e:
                    print("Failed to move vehicle:", e)
                    continue
            else:
                start = point # transform
                target = carla.Location(float(spot[0]), float(spot[1]), 0.05)
                print("Target YAW:", spot[3])
                startnode = [point.location.x, point.location.y, np.deg2rad(point.rotation.yaw)]
                goal = [float(spot[0]), float(spot[1]), np.deg2rad(spot[3])]  # Need to change to actual goal position
                ox = [] # x position list of Obstacles [m]
                oy = [] # y position list of Obstacles [m]
                ox.append(-1.5)
                oy.append(12.9)

                ox.append(1.3)
                oy.append(12.9)

                ox.append(4.5)
                oy.append(12.9)

                ox.append(7.3)
                oy.append(12.9)

                ox.append(1.5)
                oy.append(7.4)

                ox.append(4.3)
                oy.append(7.4)

                'Visualize obstacles'
                oxl = []
                for i in range(len(ox)):
                    oxl.append(carla.Location(ox[i], oy[i], 0))

                for point in oxl:
                    env.world.debug.draw_string(point, 'x', draw_shadow=False,
                                        color=carla.Color(r=0, g=0, b=255), life_time=900,
                                        persistent_lines=True)


                if not flag:
                    parking_path = env.park(startnode, goal, ox, oy)
                    reverse_path = parking_path[::-1]

                    for node in parking_path:
                        time.sleep(0.05)
                        env.vehicle.set_transform(node)

                    time.sleep(2)
                    
                    for node in reverse_path:
                        time.sleep(0.05)
                        env.vehicle.set_transform(node)
                    
                    flag = True
                        



    cv2.destroyAllWindows()