/*

  inline Helpers functions
   - Must be small functions
  
*/
#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <vector>
#include <map>

#include "Eigen-3.3/Eigen/Dense"

#include "constants.h"
#include "Vehicle.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// Get the sign of numbers
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// convert
inline double convert_mph_to_ms(double v) {
  return v * MPH_TO_MS;
}

// Here we define the d for the center of the lane
inline double convert_lane_to_d(int lane) {
  return double(lane) * LANE_WITDH + LANE_WITDH / 2.0;
}

inline int convert_d_to_lane(double d) {
  return trunc(d / LANE_WITDH);
}


// For converting back and forth between radians and degrees.
inline constexpr double pi()    { return M_PI; }
inline double deg2rad(double x) { return x * pi()  / 180.0; }
inline double rad2deg(double x) { return x * 180.0 / pi();  }

// Euclidian distance
inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// wrap s around the track
// To keep it in range [0, MAX_S]
inline double wrap_s(double s) {
  while (s >= MAX_S) {
    s -= MAX_S;
  }
  while (s < 0.0) {
    s += MAX_S;
  }
  return s;
}


// Update all vehicles states based on sensor fusions infos
// Keep active only the one in our sight
inline void update_vehicles_state(double ego_speed, double ego_s, map<int,Vehicle>& vehicles, const vector<vector<double>>& sensor_fusion) {
  
  // Define the horizon to select vehicles to track
  double front_horizon_s = ego_speed * TRACKING_HORIZON_SECONDS;
  if (front_horizon_s < MIN_TRACKING_HORIZON_RADIUS) { front_horizon_s = MIN_TRACKING_HORIZON_RADIUS; }

  double back_horizon_s  = ego_s;
  back_horizon_s  -= front_horizon_s;
  front_horizon_s += ego_s;
 
  // Update all our vehicles
  for(int i = 0; i < sensor_fusion.size(); i++) {
    const vector<double> &vehicle = sensor_fusion[i];
    int Id = vehicle[0];
    
    vehicles.emplace(Id, Vehicle());

    Vehicle::state_def state;
    state.s  = vehicle[5];

    // To deal with the track wrapping, we need to adjust
    // vehicle s position
    if (front_horizon_s > MAX_S) { 
      if (state.s < (front_horizon_s - MAX_S)) {
        state.s += MAX_S;
      }
    }
    if (back_horizon_s < 0.0) { 
      if (state.s > (back_horizon_s + MAX_S)) {
        state.s -= MAX_S;
      }
    }
    
    // If in our horizon, update it
    if (state.s < front_horizon_s && state.s > back_horizon_s) {
      state.x  = vehicle[1];
      state.y  = vehicle[2];
      state.dx = vehicle[3];
      state.dy = vehicle[4];
      state.d  = vehicle[6];
      
      vehicles[Id].assign_new_state(state);
    }
    else {
      // if going out of our horizon, de-activate it
      vehicles[Id].activate(false);
    }
  }
}


// Give the closest distance from ego along the trajectory
// to the vehicle
inline double closest_distance_on_trajectory( const vector<double>& JMT_coeffs_s
                                            , const vector<double>& JMT_coeffs_d
                                            , const Vehicle& vehicle
                                            ) {
  double closest = 99999999.0;
  
  
}


// State used to compute a JMT trajectory
struct JMT_state {
  double v;
  double v_dot;
  double v_double_dot;
};


inline vector<double> JMT(JMT_state start, JMT_state end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T in seconds.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    */
    
    MatrixXd A = MatrixXd(3, 3);
    A <<  T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T, 5*T*T*T*T,
			    6*T  , 12*T*T ,  20*T*T*T
    ;    

    VectorXd B = VectorXd(3);	    
    B << end.v            - (start.v + start.v_dot*T + 0.5*start.v_double_dot*T*T),
         end.v_dot        - (start.v_dot + start.v_double_dot*T),
         end.v_double_dot - start.v_double_dot;

    
    VectorXd x = A.colPivHouseholderQr().solve(B);
    
    return {start.v,
            start.v_dot,
            start.v_double_dot * 0.5,
            x[0],
            x[1],
            x[2]
           };    
}

// evaluate a polynomial at x using the coeffs
inline double eval_poly(const vector<double>& coeffs, const double x) {
  double result = 0.0;
  for (int i=0; i<coeffs.size(); i++) {
    result += coeffs[i] * pow(x,i);
  }
  
  return result;
}

// Derivative a polynomial
inline vector<double> derivative_poly(const vector<double>& coeffs) {
  vector<double> dcoeffs(coeffs.size()-1);
  for (int i=1; i<coeffs.size(); i++) {
    dcoeffs[i-1] = coeffs[i] * double(i);
  }
  
  return dcoeffs;
}


struct NearestCar {
  NearestCar(): id(-1), distance(0.0) {}
  int id;
  double distance;
};


// Find the nearest car in front a position in a specific lane
// Will return the index of the vehicle if found or -1
inline NearestCar findNearestCarInFront(int lane, double pos_s, map<int,Vehicle>& vehicles) {
  // Select Target, if any in the lane
  NearestCar Ncar;
  double nearest_s = MAX_S;

  // Define a horizon to deal with the wrapping world
  double wrap_horizon_s = pos_s + HALF_MAX_S;
  wrap_horizon_s -= MAX_S;

  for (auto& v: vehicles) {
    const Vehicle& vehicle = v.second;
    if (vehicle.is_active() && vehicle.get_lane() == lane) {
      // If in my lane and in front
      double vehicle_s = vehicle.get_state().s;
      if (vehicle_s < wrap_horizon_s) {
          vehicle_s += MAX_S;
      }
      if (vehicle_s > pos_s && vehicle_s < nearest_s) {
        nearest_s = vehicle_s;
        Ncar.id = v.first;
        Ncar.distance  = vehicle_s - pos_s;
      }
    }
  }
  
  return Ncar;
}

// Find the nearest car behind a position in a specific lane
// Will return the index of the vehicle if found or -1
inline NearestCar findNearestCarBehind(int lane, double pos_s, map<int,Vehicle>& vehicles) {
  // Select Target, if any in the lane
  NearestCar Ncar;
  double nearest_s = 0.0;

  // Define a horizon to deal with the wrapping world
  double wrap_horizon_s = pos_s - HALF_MAX_S;
  wrap_horizon_s += MAX_S;

  for (auto& v: vehicles) {
    const Vehicle& vehicle = v.second;
    if (vehicle.is_active() && vehicle.get_lane() == lane) {
      // If in my lane and in front
      double vehicle_s = vehicle.get_state().s;
      if (vehicle_s > wrap_horizon_s) {
          vehicle_s -= MAX_S;
      }
      if (vehicle_s < pos_s && vehicle_s > nearest_s) {
        nearest_s      = vehicle_s;
        Ncar.id        = v.first;
        Ncar.distance  = pos_s - vehicle_s;
      }
    }
  }
  
  return Ncar;
}

//============================================
// Action cost functions

inline double costActionToLane(int my_lane, double my_s, double my_d, int delta_lane, map<int,Vehicle>& vehicles) {
  int target_lane = my_lane + delta_lane;
  
  // If going out, max cost
  if (target_lane < 0 || target_lane > 2) { return MAX_ACTION_COST; }
  
  // Find the nearest car in front and behind
  NearestCar front_target = findNearestCarInFront(target_lane, my_s, vehicles);
  NearestCar back_target  = findNearestCarBehind(target_lane, my_s, vehicles);

  // Init the cost in a way that changing lane has a cost
  // So we really need to gain something from changing
  double cost = 0.0;
  //if (delta_lane != 0) { cost += 3.0 / MAX_SPEED; }

  // Use the speed of the front car to add to the cost
  // Also if not enough buffer return almost impossible cost
  // But the action might still be possible.
  if (front_target.id != -1) {
    if (delta_lane != 0 && front_target.distance < MIN_BUFFER) { return MAX_ACTION_COST - 1.0; }
    
    // Add speed cost
    const Vehicle::state_def& state = vehicles[front_target.id].get_state();
    double speed = sqrt(state.dx*state.dx + state.dy*state.dy);
    cost += (MAX_SPEED - speed) / MAX_SPEED;

    // Add distance cost
    cost += exp(-front_target.distance / MIN_BUFFER);

  }
  
  // Use the position from behind to add to cost, closer => more cost
  // Only if we change lane
  if (delta_lane != 0 && back_target.id != -1) {
    if (back_target.distance < MIN_BUFFER) { return MAX_ACTION_COST - 1.0; }
  }

  // Add as well the distance on d to reach that lane.
  double target_d = convert_lane_to_d(target_lane);
  cost += 0.15 * fabs(target_d - my_d) / (1.5 * LANE_WITDH);
  
  return cost;
}
//============================================






#endif
