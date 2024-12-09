# UAV Trajectory Tracking and Obstacle Avoidance via Control Barrier Functions

This project involves Unmanned Aerial Vehicle (UAV) control to ensure Trajectory Tracking and Obstacle Avoidance using Control Barrier Functions.
More info on the mathematical model and control technique can be found in the [report](Report.pdf).
A video of the Matlab Animation is available at this [link](https://youtu.be/bqyiTBsz_MI). A renderization of the silmulation made with SImulink and Unreal Engine is also available at this [link](https://youtu.be/Eob2qzw_-Bo)

The work done is generalizable to any scenario, with any given reference trajectory and set of obstacles (both static and dynamic). However, four scenarios were considered and are available for download in each folder of this repository.

The files in each folder include:
- A main Matlab simulation file (ex. UAVs_XXX.m)
- A main Matlab Animation file (ex. UAVsAnimation.m)
- A "workspace" file used for simulink animation (quadcopter.mat)
- A main Simulink animation file (Quadcopter.sxl)
** WARNING: ** The Simulink animation can only run with an Unreal Engine Plugin (follow Simulink Guide). Moreover, **such plugin is currently not available for MacOS.** Matlab files are compatible with any hardware.

**FOLDERS GUIDE**
For all folders, run the simulation script first and then the animation (either Matlab or Simulink).
## UAVs_TwoObstacles
In this simulation it is assigned an elicoidal trajectory with two obstacles traveling on a straight line.

## UAVs_NoFeasible
In this simulation it is assigned a trajectory made of straight lines with 90° sharp angles. It shows that the simplified model allows track unfeasible trajectory for the real UAV.
## UAVs_Straight
In this simulation the obstacle and the UAV travel in opposite directions on a line. It is a singular case for the methodology in use, but the singularity is avoided with an "escape maneuver" (more info in the [report](Report.pdf)).
## UAVs_MultipleObstacles
In this simulation it is assigned a complex trajectory and the UAV navigates through both fixed and moving obstacles. It is the most challenging scenario of all the above.

## UAVs_Kalman
In this simulation the control law is based only on exact measurements of the obstacle's position. Velocity and acceleration of the obstacle are instead estimated by an implemented Kalman Filter.





