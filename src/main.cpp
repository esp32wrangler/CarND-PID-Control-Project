#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.hpp"
#include <ctime>
#include <fstream>
#include "base64.h"


// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;
  std::vector<double> initial_params{0.311,0.241906,0.79};
  // delta zeroed out for taking final video (essentially turning of the Twiddler
  std::vector<double> deltas{0, 0, 0};
  Twiddle twiddler (initial_params, deltas, 0.1, 0.643671);
  
  auto params = twiddler.getParams();
  /**
   * TODO: Initialize the pid variable.
   */
  
  pid_steering.Init(params[0], params[1], params[2]);
  // simple PD controller, as setpoint error doesn't really matter
  pid_throttle.Init(0.2, 0, 0.08);
  int img_count = 0;
  
  auto t_start = std::chrono::high_resolution_clock::now();

  h.onMessage([&pid_steering, &pid_throttle, &t_start, &twiddler, &img_count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));
      
      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        auto image = j[1]["image"].get<string>();
        char buff[100];
        snprintf(buff, sizeof(buff), "%04d.jpg", img_count);
        auto decoded_image = base64_decode(image, false);
        std::ofstream imagefile (buff, std::ios::out | std::ios::binary);
        imagefile << decoded_image;
        imagefile.close();
        img_count++;

//        std::cout << "Event " << event << std::endl;
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          
          
          
          double dist = pid_steering.TotalDistance();
          if (dist > 1200)
          {
            std::cout << "Restarting after 1200 meters " << pid_steering.TotalTime() << std::endl;
            auto params = twiddler.getParams();
            if (twiddler.isGoalReached())
            {
              std::cout << "Goal is reached!!! " << twiddler.best_error << "," << params[0] << "," << params[1] << "," << params[2] << std::endl;
            }

            twiddler.success(pid_steering.TotalError());
            params = twiddler.getParams();
            pid_steering.Init(params[0], params[1], params[2]);
            std::cout << "Best error " << twiddler.best_error << " Trying " << params[0] << "," << params[1] << "," << params[2] << std::endl;
          }
          
          
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          

          double steer_value = pid_steering.GetControlValue(cte, 1, speed/2.23694);
          double target_speed = 60-(
                                    abs(steer_value) < 0.2 ? 0 : ((abs(steer_value)-0.2)*2)
                                   ); //(abs(steer_value*40));
          
          //std::cout << "steer " << steer_value << " i err " << pid_steering.GetIError() << " cte " << cte << std::endl;
          if (target_speed < 10)
          {
            target_speed = 10;
          }
          double speed_cte = speed-target_speed;
          double throttle_value = pid_throttle.GetControlValue(speed_cte, 1, 0);
          
          
          // DEBUG
          auto t_curr = std::chrono::high_resolution_clock::now();
          int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t_curr-t_start).count();
//          std::cout << elapsed << "," << cte << "," << steer_value << "," << speed_cte << "," << throttle_value << "," << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h, &pid_throttle, &pid_steering, &twiddler](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl<< std::endl<< std::endl<< std::endl;
    double dist = pid_steering.TotalDistance();
    std::cout << "Total distance driven " << dist << std::endl;

    auto params = twiddler.getParams();
    if (twiddler.isGoalReached())
    {
      std::cout << "Goal is reached!!! " << twiddler.best_error << "," << params[0] << "," << params[1] << "," << params[2] << std::endl;
    }


    if (dist > 800)
    {
      twiddler.success(pid_steering.TotalError());
    }
    else
    {
      if (dist != 0)
      {
        twiddler.failure();
      }
    }
    pid_throttle.Reset();
    pid_steering.Reset();
    params = twiddler.getParams();
    pid_steering.Init(params[0], params[1], params[2]);
    std::cout << "Best error " << twiddler.best_error << " Trying " << params[0] << "," << params[1] << "," << params[2] << std::endl;

  });

  h.onDisconnection([&h,&twiddler, &pid_steering](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
    double dist = pid_steering.TotalDistance();
    std::cout << "Total distance driven " << dist << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
