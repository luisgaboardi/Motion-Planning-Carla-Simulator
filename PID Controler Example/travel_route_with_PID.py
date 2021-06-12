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

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]

        spawn_point = carla.Transform(carla.Location(x=242.3, y=4, z=1), carla.Rotation(yaw=270))
        destination_point = carla.Transform(carla.Location(x=-65, y=-3, z=1))
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
	    
        sleep(1)

        world.get_spectator().set_transform(vehicle.get_transform())
        agent = Agent(vehicle)
        agent.get_route(spawn_point, destination_point)

        while (True):

            if agent.arrived():
                vehicle.apply_control(agent.soft_stop())
                print("Target reached, mission accomplished...")
                sleep(3)
                break

            agent.run_step()


    finally:
        print('Destruindo atores')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Feito.')


if __name__ == '__main__':
    main()
