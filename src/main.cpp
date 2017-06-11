#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  int step = 0;
  int twiddle_every = 1000;
  bool twiddle_reset = false;
  PID pid(false, "steer");
  PID pidThrottle(false, "throttle");

  //pid.Init(twiddle_every, 0.3, 0.004, 3.0, 0.2, 0.003, 1);
  //pid.Init(twiddle_every, 0.4, 0.001, 4.0, 0.2, 0.003, 1);
  // pidThrottle.Init(twiddle_every, 0.2, 0.003, 2.99943, 0.2, 0.003, 2);
  pid.Init(twiddle_every, 0.601021, 0.001, 9.2861, 0.2, 0.003, 1);
  pidThrottle.Init(twiddle_every, 0.3805, 0.00915, 9.08918, 0.2, 0.003, 2);

  h.onMessage([&pid, &pidThrottle, &step, &twiddle_every, &twiddle_reset](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          pid.UpdateError(cte);
          pidThrottle.UpdateError(cte);

          // steer value = - total error
          double steerValue = fmax(fmin(-pid.TotalError(), 1.0), -1.0);

          // here are some heuristic to decide the throttle value
          // we do that based on the current speed, the PID error and the steer value:
          // - speed is in [0, 100]. small speed -> bigger throttle
          // - small PID error -> bigger throttle; bigger PID error -> small throttle; error ideally in [-1, 1]
          // - small steer value -> big throttle; big steer value -> small throttle; steer value in [-1, 1]
          double throttleValue = 0.2 * (speed / 80) + 0.4 * fabs(pidThrottle.TotalError()) + 0.4 * fabs(steerValue);
          // scale throttle back to [-0.1, 0.9]
          throttleValue = 0.9 - fmax(fmin(throttleValue, 1.0), 0.0);

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: "
          //          << steerValue << " Throttle: " << throttleValue << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steerValue;
          msgJson["throttle"] = throttleValue; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (twiddle_reset) {
            step += 1;
            if (step == twiddle_every) {
              std::string msgReset = "42[\"reset\",{}]";
              ws.send(msgReset.data(), msgReset.length(), uWS::OpCode::TEXT);
              pid.reset();
              pidThrottle.reset();
              step = 0;
            }
          }
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
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
