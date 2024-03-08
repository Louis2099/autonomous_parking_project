"""
Example code from https://medium.com/@parthanvelanjeri/a-dip-into-carla-iii-camera-and-other-sensors-484fa039063c

"""

import carla
import random
import cv2
import numpy as np

client = carla.Client('localhost',2000)
world = client.load_world('Town05')
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
world.set_weather(weather)

bp_lib = world.get_blueprint_library() 
spawn_points = world.get_map().get_spawn_points()

vehicle_bp = bp_lib.find('vehicle.audi.etron')
ego_vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

spectator = world.get_spectator()
transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),ego_vehicle.get_transform().rotation)
spectator.set_transform(transform)

for i in range(200):  
    vehicle_bp = random.choice(bp_lib.filter('vehicle')) 
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

for v in world.get_actors().filter('*vehicle*'): 
    v.set_autopilot(True) 
ego_vehicle.set_autopilot(False) 

# Add RGB camera
camera_bp = bp_lib.find('sensor.camera.rgb') 
camera_init_trans = carla.Transform(carla.Location(x =-0.1,z=1.7)) 
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

def rgb_callback(image, data_dict):
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) #Reshaping with alpha channel
    img[:,:,3] = 255 #Setting the alpha to 255 
    data_dict['rgb_image'] = img

# We'll add all the other sensors' data into this dictionary later.
# For now, we've added the camera feed 
sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4))}

camera.listen(lambda image: rgb_callback(image, sensor_data))

while True:        
    # Output camera display onto an OpenCV Window
    cv2.imshow("RGB_Image",sensor_data['rgb_image'])
    if cv2.waitKey(1) == ord('q'):
        break