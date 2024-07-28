
# Lab 9: MPC, Model Predictive Control

> MPC is a model that predicts future state. The linear version of MPC uses a linearized bicycle model to model the F1TENTH vehicle. The nonlinear version is more complex and takes into account more variables and parameters.


Given the template given by [@ref(1)](https://github.com/f1tenth/f1tenth_lab9_template) and the reference [@ref(1)](https://github.com/f1tenth/f1tenth_planning), here is a simplified MPC demo, based on Kinematic Bicycle Model.

[lab9.webm](https://github.com/user-attachments/assets/ae12b470-a22d-4c47-8099-06938a4165ed)

There is a failed case (with waypoints_failed_case_csv), which might be solved with tuning. 

I also think it should be important to reimplement MPC in roscpp, helping me better understand the whole picture. However, this would be marked WIP and I might continue in the future.
