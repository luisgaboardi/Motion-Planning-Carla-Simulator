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

from pid_controller import VehiclePIDController
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

        control_vehicle = VehiclePIDController(vehicle,
                            args_lateral={'K_P': 0.58, 'K_D': 0.02, 'K_I': 0.5},
                            args_longitudinal={'K_P': 0.15, 'K_D': 0.05, 'K_I': 0.07,}
                        )

        agent = Agent(vehicle)
        route = agent.get_route(spawn_point, destination_point, True)

        for wp in route:
            while not ((abs(vehicle.get_location().x - wp.transform.location.x) <= 5
	                    and abs(vehicle.get_location().y - wp.transform.location.y) <= 5)):
                control_signal = control_vehicle.run_step(vehicle.get_speed_limit(), wp)
                vehicle.apply_control(control_signal)



    finally:
        print('Destruindo atores')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Feito.')


if __name__ == '__main__':
    main()
