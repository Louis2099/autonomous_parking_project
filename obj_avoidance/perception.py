def main():

    try:
        # Camera and lidar generation based on this: https://github.com/mjxu96/carlaviz/blob/0.9.15/examples/example.py#L63-L87

        # attach a camera and a lidar to the ego vehicle
        camera = None
        blueprint_camera = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint_camera.set_attribute("image_size_x", "640")
        blueprint_camera.set_attribute("image_size_y", "480")
        blueprint_camera.set_attribute("fov", "110")
        blueprint_camera.set_attribute("sensor_tick", "0.1")
        transform_camera = carla.Transform(carla.Location(y=+3.0, z=5.0))
        camera = world.spawn_actor(
            blueprint_camera, transform_camera, attach_to=ego_vehicle
        )
        camera.listen(lambda data: do_something(data))

        lidar = None
        blueprint_lidar = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        blueprint_lidar.set_attribute("range", "30")
        blueprint_lidar.set_attribute("rotation_frequency", "10")
        blueprint_lidar.set_attribute("channels", "32")
        blueprint_lidar.set_attribute("lower_fov", "-30")
        blueprint_lidar.set_attribute("upper_fov", "30")
        blueprint_lidar.set_attribute("points_per_second", "56000")
        transform_lidar = carla.Transform(carla.Location(x=0.0, z=5.0))
        lidar = world.spawn_actor(
            blueprint_lidar, transform_lidar, attach_to=ego_vehicle
        )
        lidar.listen(lambda data: do_something(data))
    finally:
        if lidar is not None:
            lidar.stop()
            lidar.destroy()
        if camera is not None:
            camera.stop()
            camera.destroy()


if __name__ == "__main__":
    main()
