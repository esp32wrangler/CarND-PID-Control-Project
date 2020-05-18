//
//  twiddle.hpp
//  pid
//
//  Created by Tamas Kerecsen on 2020. 05. 18..
//

#ifndef twiddle_hpp
#define twiddle_hpp

#include <vector>

class Twiddle
{
public:
  Twiddle (const std::vector<double>& initial, const std::vector<double>& delta, double goal_error_, double initial_error);
  void failure ();
  void success (double total_error);
  void update_params();
  std::vector<double> getParams();
  bool isGoalReached();
  
  double best_error;
  int twiddled_parameter;
  int twiddle_step;
  std::vector<double> parameters;
  std::vector<double> deltas;
  double goal_error;
  
};


#endif /* twiddle_hpp */
