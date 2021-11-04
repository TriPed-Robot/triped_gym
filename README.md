# Pybullet simulation of the TriPed robot


This repository provides a pybullet simulation of the [TriPed robot](https://triped-robot.github.io/docs/robot/).
It uses a simplified model with kinematic interfaces to quickly set up walking experiments.

For a more thorough simulation that is capable of computing internal forces, use the matlab simulation of the TriPed which can be found [here](https://github.com/TriPed-Robot/Matlab-Simulation)

## Getting Started
the simulation uses [trip_kinematics](https://github.com/TriPed-Robot/TriP) to calculate all kinematics.
This is done since pybullet does not support inverse kinematics for hybrid mechanisms.

TriP has to be installed before installing the simulation.
The rest of the dependencies can be installed by first cloning the repository and then installing it using pip:
```
git clone https://github.com/TriPed-Robot/TriP
cd TriP
pip install src/
```