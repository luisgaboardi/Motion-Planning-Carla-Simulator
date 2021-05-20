## Usage

The file **motion_planning.py** must be placed in: carla-simulator/PythonAPI/examples. <br>
The file **my_agent.py** must be placed in: carla-simulator/PythonAPI/carla/agents/navigation

To run:

```
python3 motion_planning.py
```

If you want to run with other vehicles in the simulation, open another terminal in the "examples" folder and enter the following command:
(Regarding the number of vehicles, respect the hardware limitation of your computer):

```
python3 spawn_npc.py -n 100 -w 0
```
