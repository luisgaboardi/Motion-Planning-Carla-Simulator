from pid_controller import VehiclePIDController


def equal_location(first_waypoint, second_waypoint):
	if((abs(first_waypoint.transform.location.x - second_waypoint.transform.location.x) <= 0.5
	and abs(first_waypoint.transform.location.y - second_waypoint.transform.location.y) <= 0.5)):
		return True

	return False

class Agent():

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.spawn_point = None
        self.destination_point = None
        self.route_planner = None
        self.route = []

    def get_route(self, spawn_point, destination_point, debug=True):
        self.spawn_point = spawn_point
        self.destination_point = destination_point
        self.route_planner = RoutePlanner(self.world, self.spawn_point, self.destination_point)
        return self.route_planner.generate_route(debug=debug)


class RoutePlanner():

    def __init__(self, world, start, end):
        self.world = world
        self.map = world.get_map()
        self.start = start
        self.end = end

    def generate_route(self, debug=True):
        start_waypoint = self.world.get_map().get_waypoint(self.start.location)
        end_waypoint = self.world.get_map().get_waypoint(self.end.location)
        route = []

        wp = start_waypoint.next(1)[0]
        while not equal_location(wp, end_waypoint):
            wp = wp.next(1)[0]
            route.append(wp)

        return route