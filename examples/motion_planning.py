# Imports para o Carla
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

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass


from agents.navigation.unb_agent import Agent

"""
Esse script consiste na implementação de alguns módulos de veículos autônomos:
    - Controladores PID para controle longitudinal e lateral
    - Alteração de rota dinamicamente mediante tratamento de sinal de
    um sensor de obstáculo posicionado na frente do véiculo.
Com isso, o veículo sai de um ponto inicial, desvia de dois obstáculos
mudando de faixa e detectando um semáforo vermelho, para antes do cruzamento
"""

def main():
    actor_list = []

    try:
        # Conecta cliente à simulação
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Configura a simulação através do cliente
        world = client.get_world()
        _map = world.get_map()
        settings = world.get_settings()
        """
        No modo síncrono configurado abaixo, o servidor espera um "tick" do cliente,
        que é uma mensagem de "pronto para prosseguir", antes de atualizar para o próximo
        passo da simulação. Na prática, isso significa que a simulação espera os cálculos
        realizados pelo cliente para prosseguir.
        """
        settings.synchronous_mode = True
        """
        A configuração abaixo permite a definição de um
        intervalo fixo entre os "passos" da simulação.
        Se setado para 0.022, acontecerão aproximadamente
        45 frames por segundo simulado
        """
        settings.fixed_delta_seconds = 0.022
        world.apply_settings(settings)        
        
        # Spawn do ego veículo e escolha do ponto de destino
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('bmw')[0]
        spawn_point = _map.get_spawn_points()[64]
        destination_point = _map.get_spawn_points()[31]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)
        world.tick()


        # Spawn primeiro obstáculo
        obstacle_bp = blueprint_library.filter('vehicle.audi.a2')[0]
        obstacle_spawn_point = _map.get_spawn_points()[62]
        obstacle = world.spawn_actor(obstacle_bp, obstacle_spawn_point)
        actor_list.append(obstacle)

        # Spawn segundo obstáculo
        obstacle_spawn_point = carla.Transform(carla.Location(x=-88.056326, y=-48.930733, z=0.930733), carla.Rotation(pitch=0.000000, yaw=89.787674, roll=0.000000))
        obstacle2 = world.spawn_actor(obstacle_bp, obstacle_spawn_point)
        actor_list.append(obstacle2)

        world.tick()

        # Cria agente e o vincula ao ego veículo 
        agent = Agent(vehicle, ignore_traffic_light=False)
        actor_list.append(agent._camera)
        actor_list.append(agent.obstacle_sensor)
        # Gera rota
        agent.set_route(spawn_point.location, destination_point.location)

        # Gameloop
        while not agent.arrived():
            world.tick()
            world.get_spectator().set_transform(agent._camera.get_transform())
            # Gera o comando de controle ao veículo
            control = agent.run_step(speed=(vehicle.get_speed_limit())) or agent.emergency_stop()
            vehicle.apply_control(control)
            # Visualização da rota
            agent.show_path(distance=int(agent.get_speed(vehicle)/2))

    finally:
        print("Destino alcançado!")
        print('Destruindo Atores')
        # Parar sensores ativos pois eles não param automaticamente ao fim da execução
        agent.obstacle_sensor.stop()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Done.')

        world.tick()

        # Desabilita modo síncrono para permitir movimentação da tela
        settings.synchronous_mode = False
        world.apply_settings(settings)


if __name__ == '__main__':
    main()