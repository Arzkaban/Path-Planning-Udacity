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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event


    // ***  Initial state of the Vehicle  ***
    // Initialized vehicle at middle lane
    static int lane = 1;
    // Intial reference velocity
    static double ref_vel = 0.0; // mph
    // Define safe distance
    double sf_dis = 30.0;

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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          //*** Start

          // Get the reminding previous path size
          int prev_size = previous_path_x.size();

          // Preventing collitions.
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          // 1. Prediction : Evaluating Ground Truth(Vehicles) information.
          bool Obj_ahead = false;
          bool Obj_left = false;
          bool Obj_right = false;
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            float d = sensor_fusion[i][6];
            int Obj_lane = -1;
            // Find out ith Object location(in which lane)
            if ( d >= 0 && d < 4 )
            {
              Obj_lane = 0;
            }
            else if ( d >= 4 && d < 8 )
            {
              Obj_lane = 1;
            }
            else if ( d >= 8 && d <= 12 )
            {
              Obj_lane = 2;
            }
            else
            {
              continue;
            }
            // Extract ith Vehicle speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double Obj_speed = sqrt(vx*vx + vy*vy);
            double Obj_s = sensor_fusion[i][5];
            // Predict Object position to compensate time different between Sensor Fusion(current) and previous path end(Future)
            Obj_s += ((double)prev_size * 0.02* Obj_speed);

            if (Obj_lane==lane)
            {
              // Object and ego Vehicle in same lane
              Obj_ahead |= (Obj_s >= car_s && Obj_s - car_s <= sf_dis);
            } 
            else if ( Obj_lane - lane == -1 )
            {
              // Object adjacent to ego Vehicle at left
              Obj_left |= (car_s - sf_dis <= Obj_s && car_s + sf_dis >= Obj_s);
            } 
            else if ( Obj_lane - lane == 1 )
            {
              // Object adjacent to ego Vehicle at right
              Obj_right |= (car_s - sf_dis <= Obj_s && car_s + sf_dis >= Obj_s);
            }
          }

          // 2. Behavior Planning
//          double speed_diff = 0;
          const double MAX_SPEED = 49.5;//mph
          const double MAX_ACC = .224;
          bool change_lane = false;
          if ( Obj_ahead )
          { // Object in front ego vehicle
              change_lane = true;
              if ( !Obj_left && lane != 0 )
              {
                  // Empty space left side & ego vehicle not at far left lane
                  lane--; // Change lane left
              }
              else if ( !Obj_right && lane != 2 ){
                  // Empty space right side & ego vehicle not at far right lane
                  lane++; // Change lane right
              }
              else
                  {
                      ref_vel -= MAX_ACC;
                      change_lane = false;
                  }
          }
          else
              {
              if ( lane != 1 )
              { // if ego vehicle not on center lane
                  if ( ( lane == 0 && !Obj_right ) || ( lane == 2 && !Obj_left ) )
                  {
                      lane = 1; // Back to center
                              change_lane = true;
                  }
              }
//              if ( ref_vel < MAX_SPEED )
//              {
//                  ref_vel += MAX_ACC;
//              }
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // are there remaining previous points
          if ( prev_size < 2 ) {
              // Less than 2 point remaining
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          else
              {
              // for continuity reason using last 2 points
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          // Initialize up target points in the future
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // shift and rotate to make math easy
          for ( int i = 0; i < ptsx.size(); i++ )
          {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Output path points from previous path for continuity
          for ( int i = 0; i < prev_size; i++ )
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate distance y position on 30 m ahead
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for( int i = 1; i < 50 - prev_size; i++ )
          {
//              ref_vel += speed_diff;
              if ( ref_vel > MAX_SPEED )
              {
                  ref_vel -= MAX_ACC;
              }
              else if ( ref_vel < MAX_SPEED  && Obj_ahead == false)
              {
                  ref_vel += MAX_ACC;
              }
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

          //*** End


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