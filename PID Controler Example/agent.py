from route_planner import RoutePlanner


class Agent():

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.spawn_point = None
        self.destination_point = None
        self.route_planner = None

    def get_route(self, spawn_point, destination_point, debug=True):
        self.spawn_point = spawn_point
        self.destination_point = destination_point
        self.route_planner = RoutePlanner(self.world, self.spawn_point, self.destination_point)
        return self.route_planner.generate_route(debug=debug)