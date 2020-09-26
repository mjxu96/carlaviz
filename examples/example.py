
import carla
import random
from carla_painter import CarlaPainter

def do_something(data):
    pass

def main():
    try:
        # initialize one painter
        painter = CarlaPainter('localhost', 8089)

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # set synchronous mode
        previous_settings = world.get_settings()
        world.apply_settings(carla.WorldSettings(
            synchronous_mode=True,
            fixed_delta_seconds=1.0 / 30.0))

        # randomly spawn an ego vehicle and several other vehicles
        spawn_points = world.get_map().get_spawn_points()
        blueprints_vehicles = world.get_blueprint_library().filter("vehicle.*")

        ego_transform = spawn_points[random.randint(0, len(spawn_points) - 1)]
        other_vehicles_transforms = []
        for _ in range(3):
            other_vehicles_transforms.append(spawn_points[random.randint(0, len(spawn_points) - 1)])

        blueprints_vehicles = [x for x in blueprints_vehicles if int(x.get_attribute('number_of_wheels')) == 4]
        # set ego vehicle's role name to let CarlaViz know this vehicle is the ego vehicle
        blueprints_vehicles[0].set_attribute('role_name', 'ego') # or set to 'hero'
        batch = [carla.command.SpawnActor(blueprints_vehicles[0], ego_transform).then(carla.command.SetAutopilot(carla.command.FutureActor, True))]
        results = client.apply_batch_sync(batch, True)
        if not results[0].error:
            ego_vehicle = world.get_actor(results[0].actor_id)
        else:
            print('spawn ego error, exit')
            ego_vehicle = None
            return

        other_vehicles = []
        batch = []
        for i in range(3):
            batch.append(carla.command.SpawnActor(blueprints_vehicles[i + 1], other_vehicles_transforms[i]).then(carla.command.SetAutopilot(carla.command.FutureActor, True)))

        # set autopilot for all these actors
        ego_vehicle.set_autopilot(True)
        results = client.apply_batch_sync(batch, True)
        for result in results:
            if not result.error:
                other_vehicles.append(result.actor_id)

        # attach a camera and a lidar to the ego vehicle
        camera = None
        blueprint_camera = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint_camera.set_attribute('image_size_x', '640')
        blueprint_camera.set_attribute('image_size_y', '480')
        blueprint_camera.set_attribute('fov', '110')
        blueprint_camera.set_attribute('sensor_tick', '0.1')
        transform_camera = carla.Transform(carla.Location(y=+3.0, z=5.0))
        camera = world.spawn_actor(blueprint_camera, transform_camera, attach_to=ego_vehicle)
        camera.listen(lambda data: do_something(data))

        lidar = None
        blueprint_lidar = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        blueprint_lidar.set_attribute('range', '30')
        blueprint_lidar.set_attribute('rotation_frequency', '10')
        blueprint_lidar.set_attribute('channels', '32')
        blueprint_lidar.set_attribute('lower_fov', '-30')
        blueprint_lidar.set_attribute('upper_fov', '30')
        blueprint_lidar.set_attribute('points_per_second', '56000')
        transform_lidar = carla.Transform(carla.Location(x=0.0, z=5.0))
        lidar = world.spawn_actor(blueprint_lidar, transform_lidar, attach_to=ego_vehicle)
        lidar.listen(lambda data: do_something(data))

        # tick to generate these actors in the game world
        world.tick()

        # save vehicles' trajectories to draw in the frontend
        trajectories = [[]]

        while (True):
            world.tick()
            ego_location = ego_vehicle.get_location()
            trajectories[0].append([ego_location.x, ego_location.y, ego_location.z])

            # draw trajectories
            painter.draw_polylines(trajectories)

            # draw ego vehicle's velocity just above the ego vehicle
            ego_velocity = ego_vehicle.get_velocity()
            velocity_str = "{:.2f}, ".format(ego_velocity.x) + "{:.2f}".format(ego_velocity.y) \
                    + ", {:.2f}".format(ego_velocity.z)
            painter.draw_texts([velocity_str],
                        [[ego_location.x, ego_location.y, ego_location.z + 10.0]], size=20)


    finally:
        if previous_settings is not None:
            world.apply_settings(previous_settings)
        if lidar is not None:
            lidar.stop()
            lidar.destroy()
        if camera is not None:
            camera.stop()
            camera.destroy()
        if ego_vehicle is not None:
            ego_vehicle.destroy()
        if other_vehicles is not None:
            client.apply_batch([carla.command.DestroyActor(x) for x in other_vehicles])

if __name__ == "__main__":
    main()
