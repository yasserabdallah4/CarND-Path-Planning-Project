#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>
#include <chrono>
#include <thread>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Car's lane. Stating at middle lane.
  int lane = 1;

  // Reference velocity.
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int prev_path_size = previous_path_x.size();

          // This following code will help us prevent collisions.
          if (prev_path_size > 0)
          {
            car_s = end_path_s;
          }

          /////////////////////// Prediction code part. ////////////////////////
          bool car_on_front = false;
          bool car_on_left = false;
          bool car_on_right = false;

          int FIRST_LANE_START = 0;
          int FIRST_LANE_END = 4;
          int SECOND_LANE_END = 8;
          int THIRD_LANE_END = 12;
          int THIRTY_AHEAD = 30;

          // Get ref_v to use
          for (int itr = 0; itr < sensor_fusion.size(); itr++)
          {
            // Car is in my lane ??
            float d = sensor_fusion[itr][6];
            int car_lane = -1;
            // IS car on saem lane as we are ?
            if (d > FIRST_LANE_START && d < FIRST_LANE_END)
            {
              car_lane = 0;
            }
            else if (d > FIRST_LANE_END && d < SECOND_LANE_END)
            {
              car_lane = 1;
            }
            else if (d > SECOND_LANE_END && d < THIRD_LANE_END)
            {
              car_lane = 2;
            }
            // bad case
            if (car_lane < 0)
            {
              continue;
            }
            // determine car's speed.
            double vx = sensor_fusion[itr][3];
            double vy = sensor_fusion[itr][4];
            double check_car_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[itr][5];

            check_car_s += ((double)prev_path_size * 0.02 * check_car_speed);

            if (car_lane == lane)
            {
              // The car is in our lane.
              car_on_front |= check_car_s > car_s && check_car_s - car_s < THIRTY_AHEAD;
            }
            else if (car_lane - lane == -1)
            {
              // The car is on left
              car_on_left |= car_s - THIRTY_AHEAD < check_car_s && car_s + THIRTY_AHEAD > check_car_s;
            }
            else if (car_lane - lane == 1)
            {
              // The car is on right
              car_on_right |= car_s - THIRTY_AHEAD < check_car_s && car_s + THIRTY_AHEAD > check_car_s;
            }
          }

          //////////////////////// Behavior code. ////////////////////////
          double diff_speed = 0;
          const double MAX_SPEED = 49.5;
          const double MAX_ACC = .224;
          if (car_on_front)
          { // Car is ahead
            if (!car_on_left && lane > 0)
            {
              // No car is on the left and there exists a left lane.
              lane--; // Lane change to left.
            }
            else if (!car_on_right && lane != 2)
            {
              // No car is on the right and there exists a right lane.
              lane++; // Lane change to right
            }
            else
            {
              diff_speed -= MAX_ACC;
            }
          }
          else
          {
            if (lane != 1)
            { // If not on center lane.
              if ((lane == 0 && !car_on_right) || (lane == 2 && !car_on_left))
              {
                lane = 1; // So, get back to center.
              }
            }
            if (ref_vel < MAX_SPEED)
            {
              diff_speed += MAX_ACC;
            }
          }

          //////////////////////// Trajectory code part ////////////////////////

          // List of widely spaces (x,y) waypoints, evenly spaced at 30m. Later, these waypoints are interpolates
          // with spine to fill more points that control speed (hence jerk and acc.).
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference for x,y and yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Car is just starting 1st time.
          if (prev_path_size < 2)
          {
            // Useing 2 points that make the path tangent to the car.
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];

            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Using the 2 points which make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Add in frenet, evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Converting coordinates to car's local coordinates.
          for (int i = 0; i < ptsx.size(); i++)
          {

            // shifting car's reference angle to ZERO degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create a spline.
          tk::spline s;

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_vals_x;
          vector<double> next_vals_y;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // Start with all of the previous path points from last time
          for (int i = 0; i < prev_path_size; i++)
          {
            next_vals_x.push_back(previous_path_x[i]);
            next_vals_y.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // Fill up rest of path planner after filling it with previous points. Here we will always output the 50 points
          for (int i = 1; i < 50 - prev_path_size; i++)
          {
            ref_vel += diff_speed;
            if (ref_vel > MAX_SPEED)
            {
              ref_vel = MAX_SPEED;
            }
            else if (ref_vel < MAX_ACC)
            {
              ref_vel = MAX_ACC;
            }
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point_val = x_add_on + target_x / N;
            double y_point_val = s(x_point_val);

            x_add_on = x_point_val;

            double x_ref_val = x_point_val;
            double y_ref_val = y_point_val;

            // Rotate back to normal after rotating it earlier
            x_point_val = (x_ref_val * cos(ref_yaw) - y_ref_val * sin(ref_yaw));
            y_point_val = (x_ref_val * sin(ref_yaw) + y_ref_val * cos(ref_yaw));

            x_point_val += ref_x;
            y_point_val += ref_y;

            next_vals_x.push_back(x_point_val);
            next_vals_y.push_back(y_point_val);
          }

          json msgJson;

          msgJson["next_x"] = next_vals_x;
          msgJson["next_y"] = next_vals_y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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