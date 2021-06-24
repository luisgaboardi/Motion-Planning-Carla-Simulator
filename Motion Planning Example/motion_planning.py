import glob
import os
import sys
from time import sleep

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla


from agent import Agent


def main():
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.025
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]

        spawn_point = world.get_map().get_spawn_points()[64]
        destination_point = world.get_map().get_spawn_points()[29]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        world.get_spectator().set_transform(vehicle.get_transform())

        obstacle_spawn_point = world.get_map().get_spawn_points()[110]
        obstacle = world.spawn_actor(vehicle_bp, obstacle_spawn_point)
        actor_list.append(obstacle)
        
        world.tick()

        agent = Agent(vehicle, ignore_traffic_light=False)
        actor_list.append(agent._camera)
        actor_list.append(agent.obstacle_sensor)
        agent.get_route(spawn_point, destination_point)

        world.tick()
        world.get_spectator().set_transform(agent._camera.get_transform())

        while not agent.arrived():
            world.tick()
            world.get_spectator().set_transform(agent._camera.get_transform())

            agent.update_information()
            control = agent.run_step(debug=False)
            vehicle.apply_control(control)
            agent.show_path(distance=15)

    finally:
        print("Target reached, mission accomplished...")
        vehicle.apply_control(agent.soft_stop())

        print('Destroying actors')
        agent.obstacle_sensor.stop()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Done.')

        world.tick()

        # Desabilita modo síncrono para permitir movimentação da tela
        settings.synchronous_mode = False
        world.apply_settings(settings)


if __name__ == '__main__':
    main()
