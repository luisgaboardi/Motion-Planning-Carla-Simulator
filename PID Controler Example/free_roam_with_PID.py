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


def main():

    #### Variáveis auxiliares ####
    # Lista de atores, registra cada novo ator criado e é destruída no fim da execução
    actor_list = []

    try:
        # Cria e conecta um cliente ao servidor
        # Por padrão, o servidor é localhost e usa a port 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Varíavel explícita para o Mundo, onde temos acesso ao mapa, blueprints etc.
        #world = client.load_world("Town06")
        world = client.get_world()

        # Varíavel explícita à Biblioteca de Blueprints, daqui tiramos o modelo do carro
        # e temos acesso aos sensores
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]
        spawn_point = carla.Transform(carla.Location(x=242.3, y=4, z=1), carla.Rotation(yaw=270))

        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)

        world.get_spectator().set_transform(vehicle.get_transform())

        control_vehicle = VehiclePIDController(vehicle,
                            args_lateral={'K_P':0.5, 'K_D':0.3, 'K_I':0.13},
                            args_longitudinal={'K_P':0.8, 'K_D':0.3, 'K_I':0.03}
                        )

        while True:
            waypoints = world.get_map().get_waypoint(vehicle.get_location())
            waypoint = waypoints.next(1)[0]
            control_signal = control_vehicle.run_step(25, waypoint)
            vehicle.apply_control(control_signal)


    finally:
        print('Destruindo atores')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Feito.')


if __name__ == '__main__':
    main()