# Carla imports
import glob
import os
import sys
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

try:
    sys.path.append(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation import pid_controller as pid
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

"""
Esse arquivo implementa um Agente, que é o responsável por dirigir o veículo 
e por tomar as decisões de controle por meio da análise do ambiente ao seu redor 
"""


class Agent():

    def __init__(self, vehicle, ignore_traffic_light=False):
        self.vehicle = vehicle
        self.ignore_traffic_light = ignore_traffic_light
        self.world = vehicle.get_world()
        self.map = self.world.get_map()
        self.control_vehicle = pid.VehiclePIDController(vehicle, max_throttle=1,
                                                        args_lateral={
                                                            'K_P': 0.58, 'K_D': 0.4, 'K_I': 0.5},
                                                        args_longitudinal={
                                                            'K_P': 0.15, 'K_D': 0.05, 'K_I': 0.07}
                                                        )
        # Relacionados à rota
        self.spawn_location = None
        self.destination_location = None
        self.dao = GlobalRoutePlannerDAO(self.map, 2.0)
        self.grp = GlobalRoutePlanner(self.dao)
        self.grp.setup()
        self.route = []

        # Relacionados aos obstáculos e sensores
        self.obstacle_info = {'distance': None, 'actor': None}
        self.emergency_brake_distance = 3
        self.dynamic_brake_distance = 0
        self.previous_obstacle = None
        self.status = ""

        # Sensores
        # Sensor de colisão sendo usado para definir a posição da câmera do simulador
        # Não é usado de fato como sensor e seus dados não são tratados
        self.camera_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.camera_transform = carla.Transform(
            carla.Location(x=-5.5, z=3.5), carla.Rotation(pitch=345))
        self._camera = self.world.spawn_actor(
            self.camera_bp, self.camera_transform, attach_to=vehicle)

        # Sensor de obstáculos
        # É uma implementação de um sensor de radar que já identifica o obstáculo
        # e a distância até ele
        self.obstacle_sensor_bp = self.world.get_blueprint_library().find(
            'sensor.other.obstacle')
        self.obstacle_sensor_bp.set_attribute('distance', '30.0')
        self.obstacle_sensor_bp.set_attribute('only_dynamics', 'True')
        self.obstacle_sensor_bp.set_attribute('sensor_tick', '0.1')
        self.obstacle_sensor_bp.set_attribute('hit_radius', '1.0')
        # Posicionado na frente do radiador do veículo
        self.obstacle_sensor_transform = carla.Transform(
            carla.Location(x=1.5, z=0.5), carla.Rotation())
        self.obstacle_sensor = self.world.spawn_actor(
            self.obstacle_sensor_bp, self.obstacle_sensor_transform, attach_to=vehicle)
        self.obstacle_sensor.listen(
            lambda data: self.obstacle_detection(data))

    def get_speed(self, vehicle):
        velocity = vehicle.get_velocity()
        return 3.6*math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

    def obstacle_detection(self, data):
        self.obstacle_info = {
            'distance': data.distance, 'actor': data.other_actor}

    def set_route(self, spawn_location, destination_location):
        self.spawn_location = spawn_location
        self.destination_location = destination_location
        self.route = [wp[0] for wp in self.grp.trace_route(
            self.spawn_location, self.destination_location)]

    def emergency_stop(self):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False
        return control

    def show_path(self, distance=15):
        route_lenght = len(self.route)
        for i in range(distance):
            if i < route_lenght:
                self.world.debug.draw_string(self.route[i].transform.location, 'o',
                                             draw_shadow=False, color=carla.Color(r=0, g=255, b=0),
                                             life_time=1, persistent_lines=True)

    def traffic_light_manager(self, waypoint):
        """

        """
        actual_landmarks = waypoint.get_landmarks_of_type(
            self.dynamic_brake_distance, "1000001", False)

        if actual_landmarks:
            next_traffic_light = self.world.get_traffic_light(
                actual_landmarks[0])
            if (str(next_traffic_light.get_state()) == "Red"
            or str(next_traffic_light.get_state()) == "Yellow"
            and actual_landmarks[0].distance < self.dynamic_brake_distance
            and self.ignore_traffic_light == False):
                return True
        return False

    def equal_location(self, vehicle, waypoint):
        if((abs(vehicle.get_location().x - waypoint.transform.location.x) <= 4
                and abs(vehicle.get_location().y - waypoint.transform.location.y) <= 4)):
            return True
        return False

    def arrived(self):
        return len(self.route) <= 1

    def run_step(self, speed=30):

        # Confere se o veículo está percorrendo a rota e atualiza o índice da rota
        if self.equal_location(self.vehicle, self.route[0]):
            self.route.pop(0)

        ego_vehicle_wp = self.map.get_waypoint(self.vehicle.get_location())
        current_speed = self.get_speed(self.vehicle)
        
        # Se o semáforo estiver vermelho ou amarelo, pare
        if self.traffic_light_manager(ego_vehicle_wp):
            if self.status != 'semáforo':
                self.status = 'semáforo'
                print(self.status)
            return self.emergency_stop()

        # Sem obstáculo ou obstáculo à uma distância segura
        if self.obstacle_info['actor'] == None or self.obstacle_info['distance'] > self.dynamic_brake_distance:
            if self.status != 'normal':
                self.status = 'normal'
                print(self.status)
            self.dynamic_brake_distance = (current_speed/7)**2 / 2
            return self.control_vehicle.run_step(speed, self.route[0])
        else:
            # Obstáculo na iminência de colisão
            if self.obstacle_info['distance'] <= self.emergency_brake_distance:
                if self.status != 'freia':
                    self.status = 'freia'
                    print(self.status)
                return self.emergency_stop()

            # Obstáculo à frente em distância que o veículo consegue desviar
            if self.obstacle_info['distance'] <= self.dynamic_brake_distance and self.obstacle_info['actor'].id != self.previous_obstacle:
                if self.status != 'desvia':
                    self.previous_obstacle = self.obstacle_info['actor'].id
                    self.status = 'desvia'
                    print(self.status, end=' ')

                vehicle_waypoint = self.map.get_waypoint(
                    self.vehicle.get_location())
                    # Confere se existe alguma faixa na rua em que se está para mudar na
                    # tentativa de não colidir com o osbtáculo. Se sim, gera uma rota até
                    # o destino a partir da outra faixa escolhida (se houver)
                if vehicle_waypoint.get_left_lane() and (int(vehicle_waypoint.lane_id) * int(vehicle_waypoint.get_left_lane().lane_id) > 0):
                    print('pra esquerda')
                    self.set_route(vehicle_waypoint.get_left_lane(
                    ).transform.location, self.destination_location)
                elif vehicle_waypoint.get_right_lane() and (int(vehicle_waypoint.lane_id) * int(vehicle_waypoint.get_right_lane().lane_id) > 0):
                    print('pra direita')
                    self.set_route(vehicle_waypoint.get_right_lane(
                    ).transform.location, self.destination_location)

                return self.control_vehicle.run_step(speed, self.route[0])

            else:
                return self.control_vehicle.run_step(speed, self.route[0])
