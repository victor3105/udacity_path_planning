#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
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
  while (getline(in_map_, line)) {
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

  // we start in the middle lane
  int lane = 1;
  // our velocity
  double ref_vel = 0;
  // timer is used to exclude two lane changes in a row
  unsigned int middle_lane_timer = 0;
  
  h.onMessage([&ref_vel, &lane, &middle_lane_timer, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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

          json msgJson;

          // use points from the previous path
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0)
            car_s = end_path_s;
          
          // these vectors contain information about vehicles in the adjacent lanes
          vector<vector<double>> lane_to_the_left_vehicles;
          vector<vector<double>> lane_to_the_right_vehicles;
          
          bool too_close = false;
          double nearest_car_speed = 0;
          
          // look at all the vehicles around
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            
            // if the vehicle is in our lane
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += (double)prev_size * 0.02 * check_speed;
              
              // if the vehicle is in front of us and closer than 30 m
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {
                too_close = true;
                nearest_car_speed = check_speed;
              }
            }
            // if the vehicle is in the lane to the left of us
            else if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2))
              lane_to_the_left_vehicles.push_back(sensor_fusion[i]);
            // if the vehicle is in the lane to the right of us
            else
              lane_to_the_right_vehicles.push_back(sensor_fusion[i]);
          }
              
          // consider lane change
          if (too_close)
          {
            bool feasible_left = false;
            bool feasible_right = false;
            double left_speed = 100;
            double right_speed = 100;
            double left_distance = 1000;
            double right_distance = 1000;
            // if we're in the middle lane
            if (lane == 1)
            {
              if(middle_lane_timer > 0)
                middle_lane_timer--;
              else
              {
				// check if it's possible to change lane
                feasible_left = checkLane(lane_to_the_left_vehicles, left_distance, left_speed, car_s, prev_size);
                feasible_right = checkLane(lane_to_the_right_vehicles, right_distance, right_speed, car_s, prev_size);

                if (feasible_left && feasible_right)
                {
                  // choose which lane is the best
                  double left_score = left_speed + left_distance;
                  double right_score = right_speed + right_distance;
                  if (left_score > right_score)
                    lane = 0;
                  else
                    lane = 2;
                }
                // if only one is possible
                else if (feasible_left)
                  lane = 0;
                else
                  lane = 2;
              }
            }
            // if we're in the left lane
            else if (lane == 0)
            {
              // check if it's possible to perform lane change left
              feasible_right = checkLane(lane_to_the_right_vehicles, right_distance, right_speed, car_s, prev_size);
              // if lane change is possible and the adjacent lane is faster or it's empty or distance to the nearest car is big enough
              if (feasible_right && ((right_speed > nearest_car_speed) || (lane_to_the_right_vehicles.size() == 0) || (right_distance > 50)))
              {
                // change lane
                lane = 1;
                // this timer is intended to exclude the possibility of two lane changes in a row
                middle_lane_timer = 40;
              }
            }
            else
            {
              feasible_left = checkLane(lane_to_the_left_vehicles, left_distance, left_speed, car_s, prev_size);
              // if lane change is possible and the adjacent lane is faster or it's empty or distance to the nearest car is big enough
              if (feasible_left && ((left_speed > nearest_car_speed) || (lane_to_the_left_vehicles.size() == 0) || (left_distance > 50)))
              {
                lane = 1;
                // this timer is intended to exclude the possibility of two lane changes in a row
                middle_lane_timer = 40;
              }
            }
          }
          
          // use these points to calculate spline
          vector<double> ptsx, ptsy;
          
          // reference the starting point as a current state
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if the previous path is almost empty, use the current state
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use previous path's endpoint
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // add evenly spaced points for spline generation
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++)
          {
            // perform transformation to make calculations easier
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }
          
          // create spline
          tk::spline s;
          
          // set point to calculate spline
          s.set_points(ptsx, ptsy);
          
          // points of our future trajectory
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // use points from the previous path
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          
          double x_add_on = 0.0;
          
          // fill up the rest of trajectory points
          for (int i = 1; i <= 50 - prev_size; i++)
          {
            	if (too_close)
          	{
            	ref_vel -= 0.3;
          	}
          	else if (ref_vel < 49.5)
          	{
            	ref_vel += 0.35;
          	}
            double N = target_dist / (0.02 * ref_vel / 2.24);            
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // perform inverse transformation
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}