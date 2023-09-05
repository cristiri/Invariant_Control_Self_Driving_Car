# Invariant Trajectory Tracking Control for Self-Driving Car
Code developed for the work published in "C. Tiriolo, W. Lucia -  On the Design of Control Invariant Regions for Feedback Linearized Car-Like Vehicles  - ACC - LCSS 2023"

Full paper at: https://doi.org/10.1109/LCSYS.2022.3224680


# Trajectory Tracking Problem Formulation 
Consider a self-driving car whose kinematics are described by bicycle models. Design a tracking controller with associated control invariant region for the tracking error of a feedback linearized car-like model guaranteeing stable full-state tracking while fulfilling the time-varying input constraints differential-drive robot subject to input constraints, i.e., limits on longitudinal and angular steering velocity of the car (see full paper for furthe details).

# Prerequisites 
The code was tested on Matlab 2020a environment and it requires Ellipsoidal Toolbox ET (https://www.mathworks.com/matlabcentral/fileexchange/21936-ellipsoidal-toolbox-et). 

# File Descriptions 
- Invariant_set_Trajectory_Tracking_Control_approachACC23.m: It is the main script to run in order to simulate the algorithm  developed in the paper. 
- DiffDrive.m: It's a Matlab function that implements the bicycle kinematics. It's used by the method ode45 to solve the differential equations describing the car's motion in the plane.
- Simulink_model_parameters.m: It's a script implementing all the parameters necessary to simulate the algorithm on Simulink. 
- Car_like_robot_trajectory_tracking.slx: It's a Simulink model that simulates the tracking control algorithm developed in the paper. 

# Demo (Matlab)
- Run "Invariant_set_Trajectory_Tracking_Control_approachACC23.m"

# Demo (Simulink)
- Run the script "Simulink_model_parameters.m"
- Run the Simulink model Car_like_robot_trajectory_tracking.slx (required Matlab 2020a or later versions)  
