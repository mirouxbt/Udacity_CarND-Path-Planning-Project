#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <random>
#include "json.hpp"

#include "helpers.h"
#include "Vehicle.h"
#include "CoordConverter.h"
#include "ActionCostHelpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load the map waypoint in our converter
  //string map_file_ = "../data/highway_map.csv";
  CoordConverter CConverter("../data/highway_map.csv");

  map<int,Vehicle> vehicles;
  vector<double> JMT_coeffs_s;
  vector<double> JMT_coeffs_d;

  h.onMessage([&CConverter,&vehicles, &JMT_coeffs_s,&JMT_coeffs_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // Convert the speed to ms-1
            car_speed = convert_mph_to_ms(car_speed);
            

            // Get the lane we are on
            int car_lane = convert_d_to_lane(car_d);
            
            cout << "ego "
                 << " x: "  << car_x
                 << " y: "  << car_y 
                 << " s: "  << car_s 
                 << " d: "  << car_d
                 << " ya: " << car_yaw
                 << " ds: " << car_speed  
                 << std::endl;
            
            
            /**************************************************
             * 
             * Section 1 - Update all vehicles based on sensors
             * 
             **************************************************/
            
            // Update all the vehicles detected by the sensors
            update_vehicles_state(car_speed, car_s, vehicles, sensor_fusion);
            
            
            /**************************************************
             * 
             * Section 2 - Use previous path to insert at the
             *             beginning of our new path and compute
             *             the remaining time we need to forcast
             *             as well as the starting point of our
             *             trajectory
             * 
             **************************************************/
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            JMT_state start_s, start_d;
            static bool is_trajectory_reset = true;

            // If we have a previous path, keep a portion of it
            // This is to avoid glitch during our process update
            // the car is still moving so some points may have been consumed already
            if (previous_path_x.size() !=0 ) {
              int n_to_keep = previous_path_x.size() > MAX_N_POINTS_PREVIOUS_PATH_TO_KEEP ? MAX_N_POINTS_PREVIOUS_PATH_TO_KEEP : previous_path_x.size() - 1;
              
              for (int i = 0; i <  n_to_keep; i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
              
              // Used previous coeffs to init the starting point of our trajectory
              // 1 - Compute the time of this point
              //     We used the comsumed path : Num of points - the number of points returned in previous path
              //     If the trajectory was defined totaly at the previous refresh
              //     we add the one we kept forward
              double t;
              if (!is_trajectory_reset) {
                t = (N_TRAJ_POINTS - double(previous_path_x.size())) * REFRESH_PERIOD;
              }
              else {
                 t = ((N_TRAJ_POINTS - double(previous_path_x.size())) + double(n_to_keep)) * REFRESH_PERIOD;
                 is_trajectory_reset = false;
              }
              
              // 2 - Evaluate at t the coeffs to get s,d
              start_s.v = wrap_s(eval_poly(JMT_coeffs_s, t));
              start_d.v = eval_poly(JMT_coeffs_d, t);
              
              // 3 - Deriv the coeffs and evaluate at t to get s_dot
              vector<double> dcoeffs_s = derivative_poly(JMT_coeffs_s);
              vector<double> dcoeffs_d = derivative_poly(JMT_coeffs_d);
              
              start_s.v_dot = eval_poly(dcoeffs_s, t);
              start_d.v_dot = eval_poly(dcoeffs_d, t);
              
              // 4 - Derive a second time to get s_double_dot
              start_s.v_double_dot = eval_poly(derivative_poly(dcoeffs_s), t);
              start_d.v_double_dot = eval_poly(derivative_poly(dcoeffs_d), t);
            }
            else {
              // s goal
              start_s.v = car_s;
              start_s.v_dot = car_speed;
              start_s.v_double_dot = 0.0;
              
              // d goal
              start_d.v = car_d;
              start_d.v_dot = 0.0;
              start_d.v_double_dot = 0.0;
              
              is_trajectory_reset = true;
            }


            // Define the trajectory horizon
            // If we used some previous point we have to reduce it.
            double traj_horizon_seconds;
            if (next_x_vals.size() > 1) {
              traj_horizon_seconds = TRAJECTORY_HORIZON_SECONDS - double(next_x_vals.size()) * REFRESH_PERIOD;
            }
            else {
              traj_horizon_seconds = TRAJECTORY_HORIZON_SECONDS;
            }



            /**************************************************
             * 
             * Section 3 - Define our behaviour action, in our
             *             case define which lane we should heading
             * 
             **************************************************/

            static int prev_goal_lane = car_lane;
            int goal_lane   = prev_goal_lane;
            double min_cost = 999999999.0;
            
            // Try to find a new lane once we moved to the lane
            if (prev_goal_lane == car_lane) {
              for (int delta_lane=-1; delta_lane < 2; delta_lane++) {
                // Get the cost to move to this lane
                double cost = costActionToLane(car_lane, car_s, car_d, delta_lane, vehicles);
                cout << "Cost dLane : " << delta_lane << " = " << cost << std::endl;
                // Stay in the same lane if at the min_cost
                if (cost < min_cost || (cost == min_cost && delta_lane == 0)) {
                  min_cost = cost;
                  goal_lane = car_lane + delta_lane;
                }
              }
              prev_goal_lane = goal_lane;
            }
            cout << "GL: "  << goal_lane << std::endl;



            /**************************************************
             * 
             * Section 4 - Based on the lane selected by the behaviour
             *             Define a potential end state
             * 
             **************************************************/

            // Define the potential end state of our trajectory
            // Select Target, if any in that lane
            int target_id = findNearestCarInFront(goal_lane, car_s, vehicles).id;
        
            // Define the end goal
            double accT;
            
            // Use target or not to define accT
            if (target_id != -1) {
              // What will be the potential state of the car
              Vehicle::state_def state = vehicles[target_id].state_at(traj_horizon_seconds);
              
              double goal_s = CConverter.getSD(state.x,state.y, atan2(state.dy, state.dx))[0];
              
              cout << "Lane : " << goal_lane << " Target : " << target_id << " GS: " << goal_s << std::endl;
              
              // Keep some buffer
              goal_s -= MIN_BUFFER;
              
              // If our goal before the starting point, meaning we wrapped, so we need to adjust it
              if ( goal_s < start_s.v ) { goal_s += MAX_S; }
              
              // Compute the potential acceleration to reach that point in traj_horizon_seconds
              accT = 2.0 * ( goal_s - start_s.v - start_s.v_dot * traj_horizon_seconds ) / (traj_horizon_seconds*traj_horizon_seconds);
              
              // Compute the acceleration to reach MAX_SPEED in traj_horizon_seconds
              double acc_1 = (MAX_SPEED - start_s.v_dot) / traj_horizon_seconds;
              
              // Use the smallest one
              if ( acc_1 < accT ) { accT = acc_1; }
              
            }
            else {
              accT = (MAX_SPEED - start_s.v_dot) / traj_horizon_seconds;
            }
            if (accT > MAX_ACCELERATION )   { accT = MAX_ACCELERATION; }
            
            
            JMT_state end_s, end_d;
            
            // Define the end goal for s assuming constant acceleration
            end_s.v = start_s.v + start_s.v_dot * traj_horizon_seconds +  accT * traj_horizon_seconds * traj_horizon_seconds / 2.0;
            end_s.v_dot = start_s.v_dot + accT * traj_horizon_seconds;
            end_s.v_double_dot = 0.0;

            // To define speed in normal direction use the target lane
            double speed_d = (convert_lane_to_d(goal_lane) - start_d.v) / traj_horizon_seconds;
            if (speed_d >  MAX_SPEED_N) { speed_d =  MAX_SPEED_N; }
            if (speed_d < -MAX_SPEED_N) { speed_d = -MAX_SPEED_N; }

            // Define the end goal for d assuming constant velocity
            end_d.v = start_d.v + speed_d * traj_horizon_seconds;
            if (fabs(speed_d) == MAX_SPEED_N) {
              end_d.v_dot = speed_d;
            }
            else {
              end_d.v_dot = 0.0;
            }
            
            end_d.v_double_dot = 0.0;
            

            
            /**************************************************
             * 
             * Section 5 - Generate the Jerk Minimized Trajectory
             *             and make sure were are not speeding up
             *             by reducing acceleration and regenerating
             *             a new trajectory if necessary
             * 
             **************************************************/

            // Get the coeffs of otraj_coeffs_sur jerk minimized trajectory
            JMT_coeffs_s = JMT(start_s, end_s, traj_horizon_seconds);
            // Get the coeffs of our jerk minimized trajectory
            JMT_coeffs_d = JMT(start_d, end_d, traj_horizon_seconds);
            
            // Make sure we do not generate a collision path when changing lane
            if (goal_lane != car_lane) {
              while (get_collision(JMT_coeffs_s, JMT_coeffs_d, CConverter, vehicles, traj_horizon_seconds)) {
                // Increase normal speed until 2 times the max speed
                speed_d += sgn(speed_d) * 0.2;
                cout << "Collision: " << speed_d << std::endl;
                if (fabs(speed_d) > MAX_SPEED_N * 3.0) { break; }

                // Define the end goal for d assuming constant velocity
                end_d.v = start_d.v + speed_d * traj_horizon_seconds;
                if (fabs(speed_d) >= MAX_SPEED_N) {
                  end_d.v_dot = speed_d;
                }
                else {
                  end_d.v_dot = 0.0;
                }
                
                end_d.v_double_dot = 0.0;
              }

              // New d trajectory
              JMT_coeffs_d = JMT(start_d, end_d, traj_horizon_seconds);
            }
            
            
            // Now decrease accT until we are under the speed limit
            while (has_exceed_speed(JMT_coeffs_s, JMT_coeffs_d, CConverter, vehicles, traj_horizon_seconds)) {
              // Reduce acceleration
              accT -= 0.05;
              
              // Define a new end goal on s
              end_s.v = start_s.v + start_s.v_dot * traj_horizon_seconds +  accT * traj_horizon_seconds * traj_horizon_seconds / 2.0;
              end_s.v_dot = start_s.v_dot + accT * traj_horizon_seconds;
              end_s.v_double_dot = 0.0;
              
              // New s Trajectory
              JMT_coeffs_s = JMT(start_s, end_s, traj_horizon_seconds);
            }


            /**************************************************
             * 
             * Section 6 - Based on the quintic polynomial of
             *             the trajectory, generate the final
             *             X,Y coordinates
             * 
             **************************************************/

            for(double i = 0.0; i < traj_horizon_seconds; i+=REFRESH_PERIOD)
            {
              double cur_s = wrap_s(eval_poly(JMT_coeffs_s, i));
              double cur_d = eval_poly(JMT_coeffs_d, i);
              vector<double> pos = CConverter.getXY(cur_s, cur_d);
              next_x_vals.push_back(pos[0]);
              next_y_vals.push_back(pos[1]);
            }

          	json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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
