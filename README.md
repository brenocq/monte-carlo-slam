# Monte Carlo SLAM Repo

<p align="center">
    <img src="https://storage.googleapis.com/atta-repos/monte-carlo-slam/monte-carlo-slam.gif" height="400">
</p>

This repository contains code for a Monte Carlo Simultaneous Localization and Mapping (SLAM) implemented with the Atta simulator. Currently, mapping is performed using GPS and infrared sensors, and Monte Carlo localization using the created map is partially implemented.

The robot, equipped with 8 IR sensors, operates in a 2D known map and its state is represented by a 2D position and an angle. The map is represented as a 50x50 binary image (black for obstacles, white for free space). The robot starts with particles spread around the map, updating and generating new particles based on their weights. The robot's position and angle are estimated from the particles, following which it moves to a desired position by performing A* on the map. 

## Components

Two custom atta components, RobotComponent and MapComponent, are utilized:

**RobotComponent**: Stores the Monte Carlo particles, estimated position, orientation, and path to goal.

**MapComponent**: Defines the occupancy grid and collision grid for the robot's environment. Each cell is 10cm by 10cm.

## Scripts

Three atta scripts are used: System, World, and Controller.

**Controller**: Handles the robot logic, including Monte Carlo localization, calculation of movement direction, and A* path generation.

**World**: Handles initialization of the robot component with random particles when the simulation starts, and erasing the particles when the simulation stops.

**System**: Responsible for displaying the UI for map selection, showing the state of each particle, the robot estimated state, and the A* path.

## Installation

This project requires Atta 0.4.1, which works for Linux and can be cloned with the following command. Please note that 0.4.1 is an experimental branch, not the main branch.

```sh
# Clone this repo
git clone git@github.com:brenocq/monte-carlo-slam.git
# Clone atta
git clone git@github.com:brenocq/atta.git
cd atta
git checkout gpu-parallel
./build.sh --run --static <path-to-monte-carlo-slam.atta>
```

## License

This project is licensed under the terms of the MIT license. See [LICENSE](LICENSE) for more details.

## References

This project is based on the Atta simulator. For more information on Atta, please refer to [the official Atta repository](https://github.com/brenocq/atta).

- "Monte Carlo localization for mobile robots". Dallaert et al. 1999.
- "Improved particle filter for nonlinear problems". Carpenter et al. 1999.
- "Improved particle filter for nonlinear problems". Rekleitis I. M. 2004.

## Future Development
The current version does not include the mapping functionality. Future versions will integrate a mapping system, allowing for a complete implementation of Monte Carlo SLAM. Please stay tuned for future updates.
