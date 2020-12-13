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

The following link shows a video of the simulated AV using the above implemented PID controller. The corresponding PID parameters have been manually tweaked on a fixed AV target velocity to be:

- **Kp = 0.13**
- **Ki = 0.001**
- **Kd = 1.22**

A PID controller comprises three different corrective effects. The proportial (**P**) part gains (i.e. multiplies) the current error with **Kp** and applies the outcome as corrective steering to simulated AV.  