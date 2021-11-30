# Pybullet simulation of the TriPed robot


This repository provides a pybullet simulation of the [TriPed robot](https://triped-robot.github.io/docs/robot/).
It uses a simplified model with kinematic interfaces to quickly set up walking experiments.


For a more thorough simulation that is capable of computing internal forces, use the Matlab simulation of the TriPed which can be found [here](https://github.com/TriPed-Robot/Matlab-Simulation)


The simulation is also packaged into an OpenAI gym environment for reinforcement learning.


## Getting Started
The package is made up of a pybullet TriPed simulation and a gym interface.
Both can be installed by calling
```
git clone https://github.com/TriPed-Robot/triped_gym
cd triped_gym
pip install src/
```
in the desired installation directory

## Using the Gym environment
The current base environment does not offer a reward function or environment (in the sense of ground, obstacles, etc...).
These currently have to be added by defining subclasses.
See ``` test_env.py ``` for reference.

Future additions with agents and full environments are planned as soon as they are finished (published).

