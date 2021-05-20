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


def equal_location(first_waypoint, second_waypoint):
	if((abs(first_waypoint.transform.location.x - second_waypoint.transform.location.x) <= 0.5
	and abs(first_waypoint.transform.location.y - second_waypoint.transform.location.y) <= 0.5)):
		return True

	return False



class RoutePlanner():

    def __init__(self, world, start, end):
        self.world = world
        self.map = world.get_map()
        self.start = start
        self.end = end

    def generate_route(self, debug=False):
        start_waypoint = self.world.get_map().get_waypoint(self.start.location)
        end_waypoint = self.world.get_map().get_waypoint(self.end.location)
        route = []
        
        wp = start_waypoint.next(1)[0]
        while not equal_location(wp, end_waypoint):
            wp = wp.next(1)[0]
            route.append(wp)

        if debug:
            for wp in route:
                self.world.debug.draw_string(wp.transform.location, 'o',
                                    draw_shadow=False, color=carla.Color(r=0, g=255, b=0),
                                    life_time=130, persistent_lines=True)

        return route


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

        world.get_spectator().set_transform(vehicle.get_transform())

        control_vehicle = VehiclePIDController(vehicle,
                            args_lateral={'K_P':0.8, 'K_D':0.3, 'K_I':0.13},
                            args_longitudinal={'K_P':0.8, 'K_D':0.3, 'K_I':0.03}
                        )

        planner = RoutePlanner(world, spawn_point, destination_point)
        route = planner.generate_route(debug=True)

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