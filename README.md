# Motion Planning on CARLA Simulator

## Objective

Development of a motion planning solution implemented in Python using the Carla Simulator.

## Usage

### Configuration
* The following python scripts were developed in Python 3.7.11, through AnaConda.
* The Operational System was Ubuntu 18.04
* The CARLA version was 0.9.11, downloaded following the Debian installation from (this installation guide)[https://carla.readthedocs.io/en/0.9.11/start_quickstart/].

* Processor: Pentium G4560
* Graphic Card: GTX 1050Ti
* Memory: 16Gb DDR4 2400mhz

### How to Run
* Add the contents of the 'agents' folder of this repository in the '/opt/carla-simulator/PythonAPI/agents' folder in the your local CARLA.
* Add the content of the 'examples' folder of this repository in the '/opt/carla-simulator/PythonAPI/examples' folder in the your local CARLA.
* Install the Python packages from 'requirements.txt' with Pip. For example ``` pip install -r requirements.txt ```
* In the '/opt/carla-simulator/PythonAPI/examples' folder, run ``` python motion_planning.py ```

## Acceptance criteria

### As for the research project
- The student must develop a script in Python that solves the motion planning problem for autonomous vehicles in CARLA simulator.
- The student must prepare a scientific article on motion planning and the developed solution.
- The student must present both the script and the article to the supervising professor for appropriate corrections, adjustments and eventual approval of the activities developed during the research period.
- The deadline of the activities is October 31, 2021.

### As for the trajectory
- The vehicle must be able to reach any point B from any other point B.
- The trajectory and the waypoints must be selected and positioned according to the OpenDRIVE high definition map of the region.
- The route generated to the destination must be efficient, that is, without being longer than necessary;
- The chosen route must obey the traffic laws

### As for the motion planning
- The vehicle must travel the route obeying the current traffic laws, such as:
    - Maximum speed
    - Minimum speed
    - Lane centering
    - Lane changes
    - Lane Direction
    - Traffic lights
    - Signs

- The vehicle must travel the route safely, avoiding accidents such as:
    - Collisions with other vehicles (front, side, rear)
    - Collisions with plates, traffic lights, cones and other static obstacles
    - Pedestrian or cyclist being run over

- The vehicle must travel the route comfortably, in order to allow a pleasant journey for the passenger, avoiding:
    - Abrupt braking
    - Sudden accelerations
    - Sharp curves
    - "Accelerate, brake, accelerate" typical behavior of congested traffic
