# System for automatic detection of infrastructure damage
## Overview
This project was done as part of my Master's thesis at University of Zagreb, Faculty of Electrical Engineering and Computing. 
The repository contains all the instructions and files necessary for running simulation of automatic detection on infrastructure damage in Gazebo simulator, on a model of an Osijek pedestrian bridge.

## Installation
Installation is rather simple because everything is preinstalled inside Docker container. A Linux operating system with installed [Docker containerization platform](https://www.docker.com/) is required.
### Prerequisites
To install Docker on your system execute the following command:
   ```sh
   curl https://raw.githubusercontent.com/larics/uav_ros_simulation/main/installation/dependencies/docker.sh | bash
   ```

### Installation
Now with Docker installed, we can pull Docker image with the following command:
   ```sh
   docker pull androkatanec/dipl_sim_final_v7
   ```
Start containr with executing create_container.sh bash script:
   ```sh
   curl https://raw.githubusercontent.com/AndroKatanec/System-for-automatic-detection-of-infrastructure-damage/master/create_container.sh | bash
   ```
## Usage
Now the Docker container is up and running, to start simulation navigate to ```startup/challenge``` and run:
```
./start.sh
```
This start Gazebo simulator with a model of an Osijek pedestrian bridge with randomly placed tiles with cracks on it:

![simulacija_screen](https://github.com/AndroKatanec/System-for-automatic-detection-of-infrastructure-damage/assets/73703833/7e854891-3c56-42ed-bb58-c584408996a2)
