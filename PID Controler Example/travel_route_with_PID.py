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
        settings.fixed_delta_seconds = 0.016
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]

        spawn_point = carla.Transform(carla.Location(x=242.3, y=4, z=1), carla.Rotation(yaw=270))
        destination_point = carla.Transform(carla.Location(x=-65, y=-3, z=1))
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        world.get_spectator().set_transform(vehicle.get_transform())
        
        world.tick()

        agent = Agent(vehicle)
        actor_list.append(agent._camera)
        agent.get_route(spawn_point, destination_point)

        world.tick()

        while (True):

            world.tick()
            world.get_spectator().set_transform(agent._camera.get_transform())

            if agent.arrived():
                vehicle.apply_control(agent.soft_stop())
                print("Target reached, mission accomplished...")
                sleep(3)
                break

            control = agent.run_step()
            vehicle.apply_control(control)


    finally:
        print('Destruindo atores')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Feito.')

        world.tick()

        settings.synchronous_mode = False
        world.apply_settings(settings)


if __name__ == '__main__':
    main()
