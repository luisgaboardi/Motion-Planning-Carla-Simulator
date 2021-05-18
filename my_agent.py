import carla
import math
from agents.navigation.agent import Agent
from agents.navigation.local_planner_behavior import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.tools.misc import get_speed


class BehaviorAgent(Agent):

    def __init__(self, vehicle, ignore_traffic_light=False):
        """
        Constructor method.
            :param vehicle: actor to apply to local planner logic onto
            :param ignore_traffic_light: boolean to ignore any traffic light
        """

        super(BehaviorAgent, self).__init__(vehicle)
        self.vehicle = vehicle
        self._local_planner = LocalPlanner(self)
        self._grp = None
        self._world = vehicle.get_world()
        self._map = self._world.get_map()

        # Vehicle information
        self.start_waypoint = None
        self.end_waypoint = None
        self._sampling_resolution = 4
        self.max_speed = 80
        self.speed_limit = vehicle.get_speed_limit() - 2
        self.light_id_to_ignore = -1

        self.emergency_brake_distance = 1
        self.start_brake_distance = 5

        self.dist_to_obstacle = [None, None]
        self.min_speed = 10.0
        self.tailgaiting_distance = 7
        self.ignore_traffic_light = ignore_traffic_light

        # Sensors
        ## Front
        self.collision_sensor_front_bp = self._world.get_blueprint_library().find(
            'sensor.other.obstacle')
        self.collision_sensor_front_bp.set_attribute('distance', '20.0')
        self.collision_sensor_front_bp.set_attribute('only_dynamics', 'True')
        self.collision_sensor_front_bp.set_attribute('hit_radius', '1.0')
        self.collision_sensor_front_transform = carla.Transform(
            carla.Location(x=1.5, z=0.5), carla.Rotation())
        self._collision_sensor_front = self._world.spawn_actor(
            self.collision_sensor_front_bp, self.collision_sensor_front_transform, attach_to=vehicle)
        self._collision_sensor_front.listen(
            lambda data: self.obstacle_detection(data))

        self.camera_bp = self._world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_bp.set_attribute('sensor_tick', '99')
        self.camera_transform = carla.Transform(
            carla.Location(x=-5.5, z=3.5), carla.Rotation(pitch=345))
        self._camera = self._world.spawn_actor(
            self.camera_bp, self.camera_transform, attach_to=vehicle)

    def update_information(self, world):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.

            :param world: carla.world object
        """
        self.speed = get_speed(self.vehicle)
        self.speed_limit = world.player.get_speed_limit()
        self._local_planner.set_speed(self.speed_limit)
        self.tailgaiting_distance = 7 + get_speed(self.vehicle) * 0.10

    def get_speed(self, vehicle):
        """
        Calcula o valor da velocidade de um veículo em Km/h
            :param vehicle: ator que se deseja saber a velocidade resultante
            :return: valor da velocidade do veículo em Km/h
        """
        velocity = vehicle.get_velocity()
        return 3.6*math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def set_destination(self, start_location, end_location, clean=False):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router.

            :param start_location: initial position
            :param end_location: final position
            :param clean: boolean to clean the waypoint queue
        """
        if clean:
            self._local_planner.waypoints_queue.clear()
        self.start_waypoint = self._map.get_waypoint(start_location)
        self.end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self._trace_route(self.start_waypoint, self.end_waypoint)

        self._local_planner.set_global_plan(route_trace)

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the
        optimal route from start_waypoint to end_waypoint.

            :param start_waypoint: initial position
            :param end_waypoint: final position
        """
        # Setting up global router
        if self._grp is None:
            world = self.vehicle.get_world()
            dao = GlobalRoutePlannerDAO(world.get_map(),
                                        sampling_resolution=self._sampling_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Obtain route plan
        route = self._grp.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        next_ = None
        l = len(route)
        for index, current in enumerate(route):
            if index < (l - 1):
                next_ = route[index+1]

            self.vehicle.get_world().debug.draw_line(current[0].transform.location, next_[0].transform.location,
                                                     0.2, color=carla.Color(r=0, g=255, b=0), life_time=len(route))

        return route

    def traffic_light_manager(self, waypoint):
        """
        This method is in charge of behaviors for red lights and stops.

        WARNING: What follows is a proxy to avoid having a car brake after running a yellow light.
        This happens because the car is still under the influence of the semaphore,
        even after passing it. So, the semaphore id is temporarely saved to
        ignore it and go around this issue, until the car is near a new one.

            :param waypoint: current waypoint of the agent
        """

        light_id = self.vehicle.get_traffic_light(
        ).id if self.vehicle.get_traffic_light() is not None else -1

        if str(self.vehicle.get_traffic_light_state()) == "Red" or str(self.vehicle.get_traffic_light_state()) == "Yellow":
            if not waypoint.is_junction and (self.light_id_to_ignore != light_id or light_id == -1):
                return 1
            elif waypoint.is_junction and light_id != -1:
                self.light_id_to_ignore = light_id
        if self.light_id_to_ignore != light_id:
            self.light_id_to_ignore = -1
        return 0

    def soft_stop(self):
        """
        Send an soft stop command to the vehicle

            :return: control for braking
        """
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.65
        control.hand_brake = False

        return control

    def obstacle_detection(self, data):
        self.dist_to_obstacle[0] = data.distance
        self.dist_to_obstacle[1] = data.other_actor

    def collision_avoidance(self, debug=False):
        """
        This module is in charge of warning in case of a collision.

            :param waypoint: current waypoint of the agent
            :return brake: brake control
        """

        if self.dist_to_obstacle[1] != None:
            if debug:
                print(self.dist_to_obstacle[1].type_id + " à " + "{:.2f}".format(
                    self.dist_to_obstacle[0]) + " metros a frente")
            return self.dist_to_obstacle[0], self.dist_to_obstacle[1]

        return [None, None]

    def run_step(self, debug=False):
        """
        Execute one step of navigation.
            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        control = None
        ego_vehicle_wp = self._map.get_waypoint(self.vehicle.get_location())

        # 1: Red lights and stops behavior
        if self.traffic_light_manager(ego_vehicle_wp) != 0 and not self.ignore_traffic_light:
            print('Red light\n')
            return self.soft_stop()

        # 2: Car detection behavior
        obstacle = self.collision_avoidance(debug=True)
        if obstacle[1] != None:

            # 2.1: Obstacle just ahead
            if obstacle[0] <= self.emergency_brake_distance:
                print('Emergency Stop!\n')
                control = self.emergency_stop()

            # 2.2: Obstacle close enough to brake
            elif obstacle[0] <= self.start_brake_distance:
                print('Braking\n')
                control = self._local_planner.run_step(
                    target_speed=0,
                    debug=debug)

            # 2.3: Tailgating distance and slow aproach if speed is less than the min
            elif obstacle[0] <= self.tailgaiting_distance:
                print('Tailgating\n')
                control = self._local_planner.run_step(
                    target_speed=min(max(self.min_speed, get_speed(
                        obstacle[1])+2.40), self.max_speed),
                    debug=debug)

            # 2.4: Normal speed but knowing something's ahead
            elif obstacle[0] > self.tailgaiting_distance:
                print('Aproaching\n')
                control = self._local_planner.run_step(
                    target_speed=min(self.max_speed, self.speed_limit), debug=debug)

        # 3: Normal behavior
        # Calculate controller based on no turn, traffic light or vehicle in front
        elif obstacle[1] == None:
            print('Normal\n')
            control = self._local_planner.run_step(
                target_speed=min(self.max_speed, self.speed_limit), debug=debug)

        self.dist_to_obstacle[0] = None
        self.dist_to_obstacle[1] = None

        return control
