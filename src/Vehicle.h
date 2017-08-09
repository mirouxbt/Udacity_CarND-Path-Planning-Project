/*

  Define a Vehicle class
  
*/
#ifndef VEHICLE_H
#define VEHICLE_H

#include "constants.h"
#include "helpers.h"


class Vehicle {
  public:
    
    struct state_def {double x,y,dx,dy,s,d;};
        
    Vehicle(): active(false) {}
    virtual ~Vehicle() {}
    
    void assign_new_state(const state_def& new_state);
    
    state_def state_at(double t) const;
    
    bool is_active() const { return active; }
    void activate(bool activate=true) {active=activate;}
    
    const state_def& get_state() const {return state;}
    int get_lane() const {return lane;}
    
  private:
    bool active;
    state_def state;
    int lane;
  
};

// Assign a new state
// If we were already active infer the speed
inline void Vehicle::assign_new_state(const state_def& new_state) {
  state = new_state;
  
  // Define the lane
  lane = trunc(state.d / LANE_WITDH);
  
  activate();
}

// Predict state at t, assuming constant velocity
// DO NOT UPDATE S and D
inline Vehicle::state_def Vehicle::state_at(double t) const {
  state_def nstate = state;
  
  nstate.x  += state.dx * t;
  nstate.y  += state.dy * t;
  
  return nstate;
}


#endif
