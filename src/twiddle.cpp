//
//  twiddle.cpp
//  pid
//
//  Created by Tamas Kerecsen on 2020. 05. 18..
//

#include "twiddle.hpp"
#include <iostream>

Twiddle::Twiddle (const std::vector<double>& initial_, const std::vector<double>& delta_, double goal_error_, double initial_error)
{
  parameters = initial_;
  deltas = delta_;
  goal_error = goal_error_;
  twiddled_parameter=0;
  twiddle_step = 0;
  parameters[twiddled_parameter] += deltas[twiddled_parameter];
  best_error = initial_error;
}

void Twiddle::update_params()
{
  
}
void Twiddle::failure ()
{
  switch (twiddle_step)
  {
    case 0:
      parameters[twiddled_parameter] -= 2*deltas[twiddled_parameter];
      if (parameters[twiddled_parameter] > 0)
      {
        twiddle_step += 1;
        break;
      }
      else
      {
        std::cout << "Negative parameter, falling through to next step " << std::endl;
      }
      // no break - fall through to next step
    case 1:
      parameters[twiddled_parameter] += deltas[twiddled_parameter];
      deltas[twiddled_parameter] *= 0.9;
      twiddled_parameter = (twiddled_parameter + 1) % parameters.size();
      twiddle_step = 0;
      parameters[twiddled_parameter] += deltas[twiddled_parameter];
      break;
  }
  std::cout << "Failure, trying " << twiddled_parameter << " " << parameters[0] << "," << parameters[1] << "," << parameters[2] << " " << deltas[0] << "," << deltas[1] << "," << deltas[2] << std::endl;
  

}
void Twiddle::success (double error)
{
  std::cout << "Error " << error << " best error " << best_error << std::endl;
  if (error > best_error)
  {
    failure();
    return;
  }
  best_error = error;
  deltas[twiddled_parameter] *= 1.1;
  twiddled_parameter = (twiddled_parameter + 1) % parameters.size();
  twiddle_step = 0;
  parameters[twiddled_parameter] += deltas[twiddled_parameter];
  
  std::cout << "Success, trying " << twiddled_parameter << " " << parameters[0] << "," << parameters[1] << "," << parameters[2] << " " << deltas[0] << "," << deltas[1] << "," << deltas[2] << std::endl;
}
std::vector<double> Twiddle::getParams()
{
  return parameters;
}
bool Twiddle::isGoalReached()
{
  return best_error < goal_error;
}
  
