import glob
import math
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

import queue
import numpy as np

def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


class VehiclePIDController():

    def __init__(self, vehicle, args_lateral, args_longitudinal, max_throttle=0.75, max_brake=0.3, max_steering=1.0):
        self.max_brake = max_brake
        self.max_steering = max_steering
        self.max_throttle = max_throttle

        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.long_controller = PIDLongitudinalControl(self.vehicle, **args_longitudinal)
        self.lat_controller = PIDLateralControl(self.vehicle, **args_lateral)


    def run_step(self, target_speed, waypoint):
        acceleration = self.long_controller.run_step(target_speed)
        current_steering = self.lat_controller.run_step(waypoint)
        control = carla.VehicleControl()

        # Longitudinal
        if acceleration >= 0.0:
            control.throttle = min(abs(acceleration), self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Lateral
        current_steering = self.lat_controller.run_step(waypoint)
        if current_steering >= 0:
            steering = min(self.max_steering, current_steering)
        else:
            steering = max(-self.max_steering, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalControl():
    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt = 0.03):
        self.vehicle = vehicle
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt = dt
        self.errorBuffer = queue.deque(maxlen = 10)

    def pid_controller(self, target_speed, current_speed):
        error = target_speed - current_speed
        self.errorBuffer.append(error)
        if len(self.errorBuffer) >= 2:
            de = (self.errorBuffer[-1] - self.errorBuffer[-2]) / self.dt
            ie = sum(self.errorBuffer)*self.dt
        else:
            de = 0.0
            ie = 0.0

        return np.clip(self.K_P*error + self.K_D*de + self.K_I*ie, -1.0, 1.0)

    def run_step(self, target_speed):
        current_speed = get_speed(self.vehicle)
        return self.pid_controller(target_speed, current_speed)


class PIDLateralControl():
    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt = 0.03):
        self.vehicle = vehicle
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt = dt
        self.errorBuffer = queue.deque(maxlen = 10)

    def pid_controller(self, waypoint, vehicle_transform):
        
        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(
                            x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                            y=math.sin(math.radians(vehicle_transform.rotation.yaw))
                        )
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                           v_begin.x, waypoint.transform.location.y -
                           v_begin.y, 0.0])
        dot = math.acos(np.clip(np.dot(w_vec, v_vec) / 
                                (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
        cross = np.cross(v_vec, w_vec)
        if cross[2] < 0:
            dot *= -1
        
        self.errorBuffer.append(dot)

        if len(self.errorBuffer) >= 2:
            de = (self.errorBuffer[-1] - self.errorBuffer[-2])/self.dt
            ie = sum(self.errorBuffer) * self.dt

        else:
            de = 0.0
            ie = 0.0

        return (self.K_P*dot + self.K_I*ie + self.K_D*de)

    def run_step(self, waypoint):
        return self.pid_controller(waypoint, self.vehicle.get_transform())
