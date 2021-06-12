import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

from pid_controller import VehiclePIDController
from route_planner import RoutePlanner


class Agent():

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.map = self.world.get_map()
        self.control_vehicle = VehiclePIDController(vehicle,
                                                    args_lateral={
                                                        'K_P': 0.58, 'K_D': 0.02, 'K_I': 0.5},
                                                    args_longitudinal={
                                                        'K_P': 0.15, 'K_D': 0.05, 'K_I': 0.07}
                                                    )
        self.spawn_point = None
        self.destination_point = None
        self.route_planner = None
        self.route = []

    def get_route(self, spawn_point, destination_point, debug=True):
        self.spawn_point = spawn_point
        self.destination_point = destination_point
        self.route_planner = RoutePlanner(
            self.world, self.spawn_point, self.destination_point)
        self.route = self.route_planner.generate_route(debug=debug)
        return self.route

    def soft_stop(self):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.65
        control.hand_brake = False

        return control

    def equal_location(self, vehicle, waypoint):
	    if((abs(vehicle.get_location().x - waypoint.transform.location.x) <= 0.5
	    and abs(vehicle.get_location().y - waypoint.transform.location.y) <= 0.5)):
		    return True
	    return False

    def run_step(self):
        print("Oi")
        while not self.equal_location(self.vehicle, self.route[0]): 
            control = self.control_vehicle.run_step(self.vehicle.get_speed_limit(), self.route[0])
            self.vehicle.apply_control(control)

        self.route.pop(0)

    def arrived(self):
        return len(self.route) <= 5
