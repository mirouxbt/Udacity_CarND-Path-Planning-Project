/*

  inline Action cost Helpers functions
  
*/
#ifndef ACTIONCOSTHELPERS_H
#define ACTIONCOSTHELPERS_H

#include <random>
#include <vector>
#include "constants.h"
#include "helpers.h"
#include "CoordConverter.h"
#include "Vehicle.h"

// Are we speeding up along the trajectory
inline bool has_exceed_speed(  const vector<double>& JMT_coeffs_s
                             , const vector<double>& JMT_coeffs_d
                             , const CoordConverter& CConverter
                             , const map<int,Vehicle>& vehicles
                             , double T
                            ) {
                                        
  // To get an accurate speed we need to convert in XY coordinate
  // And compute the speed manually
  vector<double> pos_0 = CConverter.getXY(eval_poly(JMT_coeffs_s, 0.0), eval_poly(JMT_coeffs_d, 0.0));
                                        
  // Get the max speed over a 0.01 period
  const double time_step = 0.01;
  for (double t = time_step; t <= T ; t += time_step) {
    // Get position at that time
    vector<double> pos_1 = CConverter.getXY(eval_poly(JMT_coeffs_s, t), eval_poly(JMT_coeffs_d, t));
    
    double speed = distance(pos_0[0], pos_0[1], pos_1[0], pos_1[1]) / time_step;
    
    if (speed > MAX_SPEED) { return true; }
    
    pos_0 = pos_1;
  }
    
  return false;
}


// Give the closest distance To the vehicle during a trajectory
inline double closest_distance_on_trajectory( const vector<double>& JMT_coeffs_s
                                            , const vector<double>& JMT_coeffs_d
                                            , const CoordConverter& CConverter
                                            , const Vehicle& vehicle
                                            , double T
                                            ) {
  double closest = 99999999.0;
  
  // sample at 1/100 T
  for (double t=0.0; t<=T; t+=T/100.0) {
    // Get our position in XY
    double cur_s = eval_poly(JMT_coeffs_s, t);
    double cur_d = eval_poly(JMT_coeffs_d, t);
    vector<double> pos_XY = CConverter.getXY(cur_s, cur_d);
    // Get the vehicle position
    Vehicle::state_def vehicle_state = vehicle.state_at(t);
    // Get the distance to
    double dist = distance(pos_XY[0], pos_XY[1], vehicle_state.x, vehicle_state.y);
    if (dist < closest) { closest=dist; }
  }
  
  return closest;
}

// Calculates the closest distance to any vehicle during a trajectory.
inline double closest_distance_to_any_vehicles( const vector<double>& JMT_coeffs_s
                                              , const vector<double>& JMT_coeffs_d
                                              , const CoordConverter& CConverter
                                              , const map<int,Vehicle>& vehicles
                                              , double T
                                              ) {
  double closest = 99999999.0;
  
  // Go thru all vehicles
  for (auto& v: vehicles) {
    double dist = closest_distance_on_trajectory(JMT_coeffs_s, JMT_coeffs_d, CConverter, v.second, T);
    if (dist < closest) { closest=dist; }
  }
  
  return closest;
}


inline bool get_collision(  const vector<double>& JMT_coeffs_s
                          , const vector<double>& JMT_coeffs_d
                          , const CoordConverter& CConverter
                          , const map<int,Vehicle>& vehicles
                          , double T
                          ) {

  // Get the nearest we get along the trajectory
  double nearest = closest_distance_to_any_vehicles(JMT_coeffs_s, JMT_coeffs_d, CConverter, vehicles, T);
  
  if (nearest < 2.5*VEHICLE_RADIUS ) { return true; }
  
  return false;
}


#endif
