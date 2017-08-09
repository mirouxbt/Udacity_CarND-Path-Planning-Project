/*
  Define all the constants

*/
#ifndef CONSTANTS_H
#define CONSTANTS_H

// The max s value before wrapping around the track back to 0
const double MAX_S      = 6945.554;
const double HALF_MAX_S = MAX_S / 2.0;

// Convertion from  MPH to MS-1
const double MPH_TO_MS = 0.447038889;

// The Lane with in meter
const double LANE_WITDH = 4.0;


// Minimum distance buffer to the front car in meter
const double MIN_BUFFER  = 15.0;
// The max speed allowed on this highway in MS-1
const double MAX_SPEED   = 49.5 * MPH_TO_MS;
const double MAX_SPEED_N = 1.0; // Assuming 4sec to change a 4m width lane
// Max acceleration permitted for the comfort of passenger in MS-2
const double MAX_ACCELERATION   = 10.0;
// Max jerk permitted for the comfort of passenger in MS-3
const double MAX_JERK         = 50.0;            



// Our trajectory horizon in seconds
const double TRAJECTORY_HORIZON_SECONDS = 1.5;
// Tracking constants
const double TRACKING_HORIZON_SECONDS = 5.0 * TRAJECTORY_HORIZON_SECONDS;
const double MIN_TRACKING_HORIZON_RADIUS = 2.0 * MIN_BUFFER;

// The refresh rate of our simulation
const double REFRESH_RATE = 50.0;
// The refresh period
const double REFRESH_PERIOD = 1.0 / REFRESH_RATE;

// Number of trajectory points generated
const double N_TRAJ_POINTS = TRAJECTORY_HORIZON_SECONDS / REFRESH_PERIOD;

const int MAX_N_POINTS_PREVIOUS_PATH_TO_KEEP = 10;


// model vehicle as circle to simplify collision detection
const double VEHICLE_RADIUS = 1.5;

// Max action cost value
const double MAX_ACTION_COST = 1000.0;

#endif
