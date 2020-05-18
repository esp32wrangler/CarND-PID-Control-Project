#include "PID.h"
#include <cmath>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {
  Reset();
}

PID::~PID() {}

void PID::Reset()
{
  p_error = 0;
  i_error = 0;
  d_error = 0;
  prev_p_error = 0;
  timer_initialized = false;
  total_distance = 0;
  total_error = 0;
  total_time = 0;
}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  total_distance = 0;
  total_error = 0;
  total_time = 0;
  /**
   * PID Coefficients
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

}

double PID::GetControlValue(double cte, double max_magnitude, double speed)
{
  UpdateError(cte, speed);
  
  double control_value = -p_error*Kp-d_error*Kd-i_error*Ki;
  
  if (abs(control_value) > max_magnitude)
  {
    control_value = std::copysign(1, control_value);
  }
  
  return control_value;
}

void PID::UpdateError(double cte, double speed) {
  /**
   * TODO: Update PID errors based on cte.
   */

  auto t_curr = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(t_curr-t_prev).count()/1000.0;
  t_prev = t_curr;
  
  double distance = 1;

  p_error = cte*distance;

  if (timer_initialized == false)
  {
    timer_initialized = true;
    d_error = 0;
    i_error += 0;
    dt = 0;
  }
  else
  {
    distance = dt*speed;
    if (speed == 0)
    {
      distance = 1;
    }
    
    d_error = (cte - prev_p_error)*distance/dt;
    i_error += cte*dt*distance;
  }

  
  double pterm = -p_error*Kp;
  double dterm = -d_error*Kd;
  double iterm = -i_error*Ki;

  //std::cout << " dt " << dt << " PID: " << pterm << "," << iterm << "," << dterm << " dist " << distance << std::endl;

  
  prev_p_error = cte;
  
  total_error += p_error * p_error;
  total_distance += distance;
  total_time += dt;
  if (speed != 0)
  {
    //std::cout << "dist: " << distance << ", total dist " << total_distance << " s " << speed << " dt " << dt << std::endl;
  }
}

double PID::GetIError() {
  return i_error;
}


double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return total_error/total_distance;  // TODO: Add your total error calc here!
}

double PID::TotalDistance() {
  return total_distance;
}

double PID::TotalTime()
{
  return total_time;
}
