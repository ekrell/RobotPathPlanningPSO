# Path Planning Using Particle Swarm Optimization
This is a C implementation of PSO for path planning in a 2D grid. A path is generated as a sequence of waypoints.

#### Files
- pso.c: Implementation of PSO algorithm. Modified from kkentzo's [Particle Swarm Optimization](https://github.com/kkentzo/pso) in C library.
- pathplanning.c: Defines the fitness function and methods for working with the ascii map. Uses pso.c for the actual PSO. This program's 'main' executes the path planning on an input map.
- execPSO.sh: A script used to simplify when MATLAB runs this program over the network and gets back the resulting path. Will require modification for your own system. See below.

#### Map
The environment is a 2D static ascii grid. The allowed values are 0 and 1, where 0 represents free space and 1 represents an obstacle. Here is an [example map](sample_map.txt) of size 100 x 100.

#### Number of Waypoints
The number of waypoints, *n*, is a parameter and affects the performance and correctness. Each coordinate value is a dimension of the optimization problem. Thus, additional waypoints increase the complexity exponentially. However, too few waypoints may not allow the required number of turns. The robot moves straight between consecutive waypoints, so the only opportunities to change direction are at the wapoints. Selecting the optimimal *n* value is an optimization problem. But a few experiments should be sufficient to select a useable value.

#### Parameters

Option | Name       | Description                 |
------ | :--------: | :-------------------------: |
-m     | map file   | path to ascii map file      |
-a     | map width  | number of columns in map    |
-b     | map height | number of rows in map       |
-c     | start x    | start position, x coord     |
-d     | start y    | start position, y coord     |
-e     | target x   | target position, x coord    |
-f     | target y   | target position, y coord    |
-n     | waypoints  | number of waypoints in path |

#### Example
  pathplanning -m sample_map.txt -a 100 -b 100 \
   -c 25 -d 44 -e 70 -f 67 -n 5

#### Remote Call Script (execPSO.sh)
When doing this project, MATLAB was used on a Windows machine, but this C code was written on Linux. The executable was hosted
on a Linux machine. MATLAB uploaded a map file over SCP, then
executed a script over SSH that executed pathplanning and returned the desired output. This is an unsophisticated hack
done due to a project deadline. 
