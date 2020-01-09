#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

// Constants
bool USE_TWIDDLE = false;
double speed_ref_high = 55.0; // mph
double speed_ref_low = 40.0; // mph
const double nom_speed = 50.0; // mph

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  double error_sum = 0.0;
  int count = 0;
  int twiddle_count = 0;
  uWS::Hub h;

  PID pid_steer;
  PID pid_speed;
  /**
   * Initialize the pid variable.
   */
  // set default variables to best guess
  pid_steer.Init(0.065, 0.0104, 0.065);
  pid_speed.Init(0.4, 0.05, 0.0);

  h.onMessage([&pid_steer, &pid_speed, &error_sum, &count, &twiddle_count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                           uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle;          

          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // Run steering pid control

          // Gain scaling based on speed
          double speed_lim = speed;
          double speed_ref;
          if (speed_lim < 20.0)
          {
            speed_lim = 20.0;
          }
          double gain_scale = 1.0 * nom_speed / speed_lim;

          pid_steer.UpdateError(cte * gain_scale);
          steer_value = -pid_steer.TotalError();

          // Run speed pid control  
          double speed_scale = 5.0;
          speed_ref = speed_ref_high - abs(angle)*speed_scale; 
          if (speed_ref < speed_ref_low)
          {
            speed_ref = speed_ref_low;            
          }
       
          double speed_error = speed_ref - speed;
          if (speed_error<0.0)
          {
            speed_error *= 0.33;
          } 
          pid_speed.UpdateError(speed_error);
          throttle = pid_speed.TotalError();
          count += 1;

          // DEBUG
          if (count % 10 == 0)
          {
            count = 0;
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                      << ", car angle: " << angle        
                      << ", Speed: " << speed << " Throttle Value: " << throttle
                      << std::endl;
          }

          // Start twidle
          if (USE_TWIDDLE == true)
          {
            int N_Twiddle = 5 * 25;

            if (speed >= speed_ref * 0.8)
            {
              // measure total error
              error_sum += cte;
              twiddle_count += 1;
              if (twiddle_count > N_Twiddle)
              {
                pid_steer.UpdateTwiddle(error_sum);
                std::cout << "\nNew Twiddle state: " << pid_steer.TwiddleState << "\n"
                          << std::endl;
                twiddle_count = 0;
                error_sum = 0.0;
              }
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}