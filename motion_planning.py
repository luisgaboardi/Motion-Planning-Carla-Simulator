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

from agents.navigation.my_agent import BehaviorAgent
from time import sleep


'''
Esse script utiliza do autor Behaviour já incluso no Carla para traçar uma rota ótima entre um ponto inicial
até um final, mantendo o carro no meio da faixa, respeitando semáforos, não batendo frontalmente em outros carros
e acompanhando a velocidade do carro da frente ao invés de ficar dando trancos
'''


#### Variáveis auxiliares ####
# Lista de atores, registra cada novo ator criado e é destruída no fim da execução
actor_list = []


##################################################
# Frenagem e velocidade atual?
# Distância e intensidade de freio confortável
# Clima e tempo
# Cenário específico

# Sensor diagonal e lateral
##################################################
# Sensor GNSS pra coletar a posição
# Diante da posição do semáforo e da do carro, calcular distância
# Detecção de semáforo com câmera
##################################################



def main():

    try:
        # Cria e conecta um cliente ao servidor
        # Por padrão, o servidor é localhost e usa a port 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Varíavel explícita para o Mundo, onde temos acesso ao mapa, blueprints etc.
        #world = client.load_world("Town06")
        world = client.get_world()

        # Definindo a partir do cliente se o servidor será síncrono ou assíncrono,
        # e com time_stamp fixo ou varíavel
        # Usaremos o modo síncrono com time_stamp fixo pela estabilidade da execução
        # e repetibilidade dos testes
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.06
        world.apply_settings(settings)

        # Varíavel explícita à Biblioteca de Blueprints, daqui tiramos o modelo do carro
        # e temos acesso aos sensores
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]

        spawn_point = carla.Transform(carla.Location(x=246.0, y=4, z=1), carla.Rotation(yaw=270))
        destination_location = carla.Location(x=-65, y=-3, z=1)
        # waypoints = world.get_map().get_spawn_points()
        # spawn_point = random.choice(waypoints)
        # destination_location = random.choice(waypoints).location

        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        actor_list.append(vehicle)  
        world.player = vehicle
        
        world.tick()

        # Coloca obstáculo no caminho de ego car
        # obstacle_transform = carla.Transform(carla.Location(x=40, y=-203.8, z=1), carla.Rotation(yaw=180))
        # obstacle_bp = blueprint_library.filter('model3')[0]
        # obstacle = world.spawn_actor(obstacle_bp, obstacle_transform)
        # actor_list.append(obstacle)
        
        # world.tick()

        agent = BehaviorAgent(vehicle, False)
        actor_list.append(agent._collision_sensor_left)
        actor_list.append(agent._collision_sensor_front)
        actor_list.append(agent._collision_sensor_right)
        actor_list.append(agent._camera)
        
        agent.set_destination(agent.vehicle.get_location(), destination_location, clean=True)

        while (True):

            world.tick()
            world.get_spectator().set_transform(agent._camera.get_transform())

            if len(agent.get_local_planner().waypoints_queue) == 0:
                vehicle.apply_control(agent.emergency_stop())
                print("Target reached, mission accomplished...")
                sleep(3)
                break

            agent.update_information(world)
            control = agent.run_step()
            vehicle.apply_control(control)

    finally:
        print('Destruindo atores')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('Feito.')

        world.tick()

        # Desabilita modo síncrono para permitir movimentação da tela
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.02
        world.apply_settings(settings)


if __name__ == '__main__':
    main()