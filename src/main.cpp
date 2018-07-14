#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <iomanip>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// clamp
double clamp(double val, double lo, double hi) { return max(lo, min(val, hi)); }

// max CTE
constexpr double cte_max = 5.0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steer;
  // TODO: Initialize the pid variable.
  // The initial values (0.1, 0.0006 and 0.5) were set using manual tuning.
  // Final values below are set after running Optimizer.
  double Kp_s = 0.116791;
  double Ki_s = 0.0007332;
  double Kd_s = 0.533709;
  //override initial values via environmental variables
  const char* env_var = getenv("STEER_INIT_PID");
  if (env_var != NULL) {
    sscanf(env_var, "%lf %lf %lf", &Kp_s, &Ki_s, &Kd_s);
  }
  cout << "Initializing steering angle PID controller.\n";
  cout << "P:" << Kp_s << " I:" << Ki_s << " D:" << Kd_s << endl;

  pid_steer.Init(Kp_s ,Ki_s, Kd_s);
  // InitOptimizer ,setting all 0 disables the optimizer
  pid_steer.InitOptimizer(0.0, 0.0, 0.0, 0.0);

  PID pid_throttle;
  /*
   * The initial values (3.0, 0.0 and 0.1) were set using manual tuning.
   * Ki = 0 is zero as we are feeding fabs(cte) as error so integral component will increase indefinitely.
   * So this is effectively a PD controller.
   * Final values below are set after running Optimizer.
   */
  double Kp_t = 3.50373;
  double Ki_t = 0.0;
  double Kd_t = 0.106742;
  //override initial values via environmental variables
  env_var = getenv("THROTTLE_INIT_PID");
  if (env_var != NULL) {
    sscanf(env_var, "%lf %lf %lf", &Kp_t, &Ki_t, &Kd_t);
  }
  cout << "Initializing throttle PID controller.\n";
  cout << "P:" << Kp_t << " I:" << Ki_t << " D:" << Kd_t << endl;

  pid_throttle.Init(Kp_t ,Ki_t, Kd_t);
  // InitOptimizer ,setting all 0 disables the optimizer
  pid_throttle.InitOptimizer(0.0, 0.0, 0.0, 0.0);

  h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;
          double throttle = 0.0;

          if (fabs(cte) >= cte_max) {
            cout << "Vehicle went off the track\n";
            return;
          }

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          */
          pid_steer.UpdateError(cte);

          /* The +ve cte implies that the car is too far to the right of the center.
          * The +ve steering_value implies steer to the right.
          * We need to steer left if the car is too far to right and viceversa.
          * Negate the value returned by controller to get correct steering value.
          */
          steer_value = -1 * pid_steer.TotalError();
          // clamp the value between -1.0 and 1.0
          steer_value = clamp(steer_value, -1.0, 1.0);

          /*
           * Update error for throttle controller.
           * For throttle/speed it does not matter whether we are too far to left/right.
           * so only look at the magnitude. Divide the cte by max_cte to restrict the input to 0 to 1.0
           */
          pid_throttle.UpdateError(fabs(cte));

          /*
           * Get the throttle value.
           * If the error is high, the vehicle should throttle down.
           */
          throttle = 1.0 - pid_throttle.TotalError();
          // clamp the value between -1.0 and 1.0
          throttle = clamp(throttle, 0.1, 0.9);

#if 1 // DEBUG_PRINTS
          static int num_updates = 0, num_laps = 0;
          static double cte_sum = 0, speed_max = 0, speed_sum = 0;
          cte_sum += fabs(cte);
          speed_sum += speed;
          speed_max = max(speed_max, speed);
          num_updates++;
          if ((num_updates % 640) == 0) { // 1 lap ~ 640 updates.
            cout << "Lap:" << num_laps++ << " Avg CTE: " << cte_sum/640 << " Avg Speed:" << speed_sum/640 << " Max Speed:" << speed_max << endl;
            num_updates = 0;
            cte_sum = 0;
            speed_sum = 0;
            speed_max = 0;
          }
          cout << fixed << setw(5) << setprecision(3);
          cout << "E:" << cte << " S:" << steer_value << " T:" << throttle << "\r" << flush;
          //std::cout << msg << std::endl;
#endif
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
