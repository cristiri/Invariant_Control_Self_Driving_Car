# Invariant Trajectory Tracking Control for Self Driving Car
Code developed for the work published in "C. Tiriolo, W. Lucia -  On the Design of Control Invariant Regions for Feedback Linearized Car-Like Vehicles  - ACC - LCSS 2023"

Full paper at: https://doi.org/10.1109/LCSYS.2022.3224680


# Trajectory Tracking Problem Formulation 
Consider a self-driving car whose kinematics are described by bicycle models. Design a tracking controller with associated control invariant region for the tracking error of a feedback linearized car-like model guaranteeing stable full-state tracking while fulfilling the time-varying input constraints differential-drive robot subject to input constraints, i.e., limits on longitudinal and angular steering velocity of the car (see full paper for furthe details).

# Prerequisites 
The code was tested on Matlab 2020a environment and it requires Ellipsoidal Toolbox ET (https://www.mathworks.com/matlabcentral/fileexchange/21936-ellipsoidal-toolbox-et). 

# File Descriptions 
- Khepera_iv_FL_RHC_traj_track.m: It is the main script to run in order to perform the experiment proposed in the paper. 
- Khepera4.m: It represents the core of the application. It is a Matlab class that implements the main communication functionalities between the tracking controller running on Matlab and the server running on Khepera IV
- STTT_control_parameters.m: It's a Matlab class defining the parameters needed by the proposed tracking controller.
- "eight_traj_generation.m" and "circular_traj_generation.m" implements the reference trajectory, an eight-shaped and a circular one, respectively.

# Demo 
- Connect KheperaIV to the host machine through Bluetooth and set the right port on the script "Khepera_iv_FL_RHC_traj_track.m".
- Run the Bluetooth server on the KheperaIV side and then, run the script Khepera_iv_FL_RHC_traj_track.m

# Videos
- https://www.youtube.com/watch?v=A0Tlbgr08tY&ab_channel=PreCySeGroup
- https://www.youtube.com/watch?v=L3wmg-pHx_4&list=PLh-6B_s-jPuT8RTDOJM96GXu4y1IBIeoC&ab_channel=PreCySeGroup
