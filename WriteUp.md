# Control
The following control methods are applied in the project:
* Standard PID control for steering
* Standard PID control for speed control
* Gain scaling control for steering control to decrease gain when speed is higher than nominal speed, to improve stability at high speeds 
* Gain scaling control for speed control where car angle is used to decrease speed reference when angle is above a threshold
*  Twiddle implemented, but is deactivated


# PID control
PID control is used to control car speed and steering.

**P** term for steering will try to bring the car to the center of the track, by steering the wheels towards center. This can lead to to oscillations when the car overshoots the center.

**D** term for steering will try to minimize the derivative of the error. This will make the car resilient towards changes in error values, and can make the car steer towards the lane direction and prevent overshoot. But it can also make the car unstable if noise is seen in the car angle.

**I** term for steering will try to reduce constant offset in eg. steering angle or other constant error terms, by integrating the constant error contributions and applying them in the steering angle. 

# Tuning
The control was tuned initialyl by hand, and using initial values found from Python Twiddle, using a higher speed reference than the default speed of 1m/s.
Then I tried using an online version of Twiddle in c++, but it turned too difficult, as I tried to update the algorithm every couple of seconds, when the lane is not very homogene. Ideally it should be run for a full round for each parameter update, but this is not possible.

Then I tuned the car further by hand, and using different control tricks, mainly gain scheduling, to make the car go as fast as possible, up to 80mph, when the lane is straight. 

The speed control is just a simple PI controller, where negative errors have a different gain, as they correspond to braking. This was chosen as speed control is a simpler task than steering.

**Final parameters**

pid_steer: Kp=0.084050, Ki= 0.0104622135, Kd=0.033756267

pid_speed: Kp=0.4, Ki=0.05

Two videos showing the car running on the track using:
1. Slow tuned control (Max speed=60mph):
![alt text](./Video/Car_PID_slow.mp4 "Slow tuning")
2. Fast tuned control (Max speed=80mph): ![alt text](./Video/Car_PID2.mp4 "Fast tuning")



