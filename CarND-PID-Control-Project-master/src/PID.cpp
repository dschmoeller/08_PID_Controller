#include "PID.h"
#include <cstdlib> 

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_; 
  Ki = Ki_; 
  Kd = Kd_;
  prev_cte = 0;
  total_error = 0;  

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   // PID errors
   p_error = cte; 
   i_error = i_error + cte; 
   d_error = cte - prev_cte; 
   // Store current cte for next iteration
   prev_cte = cte;
   // Update total error
   total_error += std::abs(cte);  
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return total_error;   // TODO: Add your total error calc here!
}

double PID::Control(){
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}