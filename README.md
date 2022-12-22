# Infinite-Horizon Stochastic Optimal Control using CEC and GPI Approaches

<p align="justify">
This project focuses on the approaches to design a control policy for a ground differential-drive robot for safely tracking a reference trajectory while avoiding obstacles in the environment. The robot follows a discrete-time kinematic motion model with motion noise modelled as a Gaussian distribution. The given environment is a map of [−3,3]2 with two circular obstacles C1 centered at [−2,−2] and C2 centered at [1,2] and with radius 0.5. The error state is defined to measure the deviation of position and orientation from the reference trajectory. The trajectory tracking problem is formulated as a discounted infinite-horizon stochastic optimal control problem. It is solved using the approaches of receding-horizon Certainty Equivalent Control (CEC) and Generalized Policy Iteration (GPI). The results of trajectory tracking performed by robot using both the approaches in the given environment are presented in the report.
</p>

## Project Report
[Sanchit Gupta, 'Infinite-Horizon Stochastic Optimal Control using CEC and GPI Approaches', ECE 276B, Course Project, UCSD](https://github.com/sanchit3103/Stochastic-optimal-control_CEC-and-GPI/blob/main/Report.pdf)

## Implementation of CEC approach in form of GIF files

#### Trajectory tracking for a set of optimal parameters after tuning while ignoring noise
<p align="center">

  <img src = "https://user-images.githubusercontent.com/4907348/209074759-047b116a-a664-485b-8510-9fe8c29cbafb.gif"/>
  
</p>

#### Trajectory tracking for a set of optimal parameters after tuning while considering noise
<p align="center">

  <img src = "https://user-images.githubusercontent.com/4907348/209078036-3ecc789d-1363-4c18-bc9f-fe20b57bb4f6.gif"/>
  
</p>

## Details to run the code

* <b> main.py: </b> Main file which should be run to execute the project. Appropriate controller will have to be chosen in this file.
* <b> utils.py: </b> Contains all the utility functions required for the project.
* <b> cec_controller.py: </b> Design and Implementation of CEC approach based control policy.
* <b> gpi_controller.py: </b> Design and Implementation of GPI approach based control policy.
