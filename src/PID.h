#ifndef PID_H
#define PID_H

#include <chrono>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);
  void Reset();

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double speed);

  double GetControlValue(double cte, double max_magnitude, double speed);
  
  double GetIError();
  
  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  double TotalDistance();
  double TotalTime();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_p_error;
  std::chrono::time_point<std::chrono::high_resolution_clock> t_prev;
  bool timer_initialized;
  double total_error;
  double total_distance;
  double total_time;
  
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H
