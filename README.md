# Pybullet simulation of the TriPed robot


This repository provides a pybullet simulation of the [TriPed robot](https://triped-robot.github.io/docs/robot/).
It uses a simplified model with kinematic interfaces to quickly set up walking experiments.

For a more thorough simulation that is capable of computing internal forces, use the matlab simulation of the TriPed which can be found [here](https://github.com/TriPed-Robot/Matlab-Simulation)

## Getting Started
The package is made up of a pybullet triped simulation and a gym interface.
Both can be installed by calling
```
git clone https://github.com/TriPed-Robot/triped_gym
cd triped_gym
pip install src/
```