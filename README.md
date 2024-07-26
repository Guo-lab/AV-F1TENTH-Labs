# AV-F1TENTH-Labs

- ### lab 1: Tutorial, Introduction to ROS2
  Getting familiar with ROS 2 workflow, understanding how to create nodes with publishers, subscribers, launch files and understanding ROS 2 package structure, files, dependencies.


- ### lab 2: Safety: AEB (Automatic Emergency Braking)
    Use Instantaneous Time to Collision (iTTC) to build a safety critical system. Based on this, the safety node could halt the car before it collides with obstacles. 


- ### lab 3: Wall Following
    Use PID Controllers to drive the vehicle automatically via wall following.
    
    > A PID controller is a way to maintain certain parameters of a system around a specified set point.
    
    The control output `u(t)` is the steering angle we want the car to drive at. The error term `e(t)` is the difference between the set point and the parameter we want to maintain around that set point, which is the difference between the desired and actual distance to the wall.

- ### lab 4: Follow the Gap
    Implement a reactive method (a gap follow algorithm) for obstacle avoidance.

- ### lab 6: Pure Pursuit
    Use Pure Pursuit control to track and go through the series of waypoints. 
    $$ \gamma = \frac{2 |y|}{L^2}$$

- ### lab 7: Motion Planning, RRT
    Use sampling based algorithms: RRT to do the local planning.
    - Converting workspace to configuration space.
    - Use occupancy grids for RRT to do obstacle avoidance.
    - After the RRT provided a path, use pure pursuit to follow that trajectory.

- ### lab 9: (old version) MPC, Model Predictive Control
    > MPC is a model that predicts future state. The linear version of MPC uses a linearized bicycle model to model the F1TENTH vehicle. The nonlinear version is more complex and takes into account more variables and parameters.

In this repo,

`run_simulator` includes the command to build and set-up the simulator.

The demos shown in each package `README.md`.