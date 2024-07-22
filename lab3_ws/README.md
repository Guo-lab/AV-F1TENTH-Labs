
# Lab 3: Wall Following

Use PID Controllers to drive the vehicle automatically via wall following.
    
> A PID controller is a way to maintain certain parameters of a system around a specified set point.
    
The control output `u(t)` is the steering angle we want the car to drive at. The error term `e(t)` is the difference between the set point and the parameter we want to maintain around that set point, which is the difference between the desired and actual distance to the wall.
