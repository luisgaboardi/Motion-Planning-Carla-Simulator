## Utilização

O arquivo **motion_planning.py** deve ser colocado em: carla-simulator/PythonAPI/examples. <br>
O arquivo **my_agent.py** deve ser colocado em: carla-simulator/PythonAPI/carla/agents/navigation

Para rodar:

```
python3 motion_planning.py
```

Caso deseje rodar com outros veículos na simulação, abra outro terminal na pasta de "examples" e digite o seguinte comando:
(Quanto a quantidade de veículos, respeite a limitação de hardware do seu computador):

```
python3 spawn_npc.py -n 100 -w 0
```