# Project 8: PID Control



## **Project Goals: **

* PID controller in C++ to maneuver the vehicle around the track!
* The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle
* There´s no minimum speed to pass



## Implementation

The implementation comprises a state of the art PID controller with predefined and fixed parameters Kp, Ki and Kd. After defining these parameters, the PID controller gets initialized. 

```c++
  // Define PID controller
  PID pid = PID(); 
  double Kp = 0.13; 
  double Ki = 0.001; 
  double Kd = 1.22;  

  // Initialize PID control parameters
  pid.Init(Kp, Ki, Kd); 
```



Once the PID controller has been initialized, the actual control function is being called iteratively within the ***onMessage( )*** function. The current tracking error is received from the simulation interface and updated within the PID controller object. Afterwards a steering command is calculated. Due to the steering actor signal is constrained to values between -1 and +1, the PID controller outcome is programmatically limited.

```c++
   // Apply PID control
   pid.UpdateError(cte); 
   steer_value = pid.Control(); 
   // Apply steer contstraints
   if (steer_value > 1){ steer_value = 1; }
   else if (steer_value < -1){ steer_value = -1; } 
```



The actual PID control law looks like this. 

```c++
double PID::Control(){
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}
```



## Reflection 

The following link shows a video of the simulated AV using the above implemented PID controller. The corresponding PID parameters have been manually tweaked on a fixed AV target velocity. Due to missing plant (model) knowledge, no analytical approaches could be applied. Also, it was not really convenient to identify the step response, so one could have applied rules of thumbs (e.g. from Ziegler and Nichols). That´s why I tweak the parameters manually. First I increased Kp until I observed oscillating AV behavior. Afterwards, I applied another rule of thumb for **Ki** to be **1/100*Kp** and **Kd** to be **10*Kp**. This led to a stable AV behavior. Based on this initial parameter set, I manually tweaked the parameters further based on analyzing the absolute track error on the simulator. The final parameters have been chosen to be: 

- **Kp = 0.13**
- **Ki = 0.001**
- **Kd = 1.22**

[]: https://github.com/dschmoeller/08_PID_Controller/blob/master/CarND-PID-Control-Project-master/AV_PID.mp4



A PID controller comprises three different corrective effects. The proportional (**P**) part gains (i.e. multiplies) the current error with **Kp** and applies the outcome as corrective steering to simulated AV. The same corrective structure applies to the other two components, Integral (**I**) and Differential (**D**) part. However, the Integral part gains the accumulated error and the Differential part deals with the error change rate. PID controllers can be nicely described in the complex s area. Due to the fact that the (simulated) AV model is not really known in this particular setting, this kind of argumentation isn´t appropriate to be applied here. That´s why the following discussion is rather a qualitative description of the respective PID control effects than an analytical one. In the context of trajectory tracking control, the Proportional part takes rather care of slight and constant tracking errors. However, the closer the AV gets to the desired trajectory, the less is the proportional corrective effect. That´s why a P controller as standalone can not reach the reference value. There´s gonna be a stationary gap, unless the plant contains a free "integrator". If not, the Integral component of the PID controller provides this in order to overcome this gap. The Differential part of this particular scenario can be seen as "curve bootstrap", which provides additional corrective steering when the track curvature is high. In general, one should be rather careful with adding a Differential component as it can lead to undesired behavior. 



In general, the PID controller is not the best approach for these kind of trajectory following problems. There are way better approaches, for instance MPC. One can also think of applying feedforward control due to the track is already known offline. Also, it would be beneficial to apply model driven approaches. By modeling the plant, one could calculate analytical optimal control parameters. Due to model uncertainties, these parameters could be further manually adapted.         