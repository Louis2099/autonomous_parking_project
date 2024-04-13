import carla

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load the map
world = client.load_world('Town05')  # Load the desired map


# Load the FBX file
fbx_file_path = r"D:\autonomous_parking_project\maps\parkinglot.fbx"
fbx_actor = None
with open(fbx_file_path, 'rb') as f:
    fbx_data = f.read()
    fbx_bp = world.get_blueprint_library().find("static.prop.mesh")
    fbx_actor = world.try_spawn_actor(fbx_bp, carla.Transform())

# Load the XODR file
xodr_file_path = r"D:\autonomous_parking_project\maps\parkinglot.xodr"
with open(xodr_file_path, 'r') as f:
    xodr_data = f.read()

carla.Map.load(xodr_data)

# Wait for a while to make sure everything is loaded
import time
time.sleep(5)

# Destroy actors and cleanup
if fbx_actor is not None:
    fbx_actor.destroy()