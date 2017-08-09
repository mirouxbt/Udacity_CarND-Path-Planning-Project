/*

  Define a converter for the 2 coordinate systems used
  The global x,y system and the frenet s,d system
  
*/
#ifndef COORDCONVERTER_H
#define COORDCONVERTER_H

#include <fstream>
#include <vector>
#include "constants.h"
#include "helpers.h"
#include "spline.h"


class CoordConverter {
  public:
    CoordConverter(const string& map_file);
    virtual ~CoordConverter() {}
    
    vector<double> getXY(const double s, const double d) const;
    vector<double> getSD(const double x, const double y, const double theta) const;
    
  private:
    int ClosestWaypoint(const double x, const double y) const;
    int NextWaypoint(const double x, const double y, const double theta) const;
  
    // raw waypoints data
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
  
    // Interpolated waypoints using a spline
    tk::spline waypoint_spline_x;
    tk::spline waypoint_spline_y;
    tk::spline waypoint_spline_dx;
    tk::spline waypoint_spline_dy;
};

inline CoordConverter::CoordConverter(const string& map_file) {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  ifstream in_map_(map_file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // Spline interpolate the map waypoints from s
  waypoint_spline_x.set_points(map_waypoints_s, map_waypoints_x);
  waypoint_spline_y.set_points(map_waypoints_s, map_waypoints_y);
  waypoint_spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  waypoint_spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
  
  // remove the last point in maps, same as 1st one
  // It is necessary for the spline interpolation but create issue
  // for our conversion from XY to SD
  map_waypoints_s.pop_back();
  map_waypoints_x.pop_back();
  map_waypoints_y.pop_back();
  map_waypoints_dx.pop_back();
  map_waypoints_dy.pop_back();
}

// Convert from frenet to world coord
inline vector<double> CoordConverter::getXY(const double s, const double d) const {
  // Get xy coordinate on the waypoint spline
  double x = waypoint_spline_x(s);
  double y = waypoint_spline_y(s);
  
  // Add the offset from d
  x += waypoint_spline_dx(s) * d;
  y += waypoint_spline_dy(s) * d;
  
  return {x,y};  
}


// Return the index of the closest waypoint to (x,y)
inline int CoordConverter::ClosestWaypoint(const double x, const double y) const {
	double closestLen = 100000.0; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < map_waypoints_x.size(); i++)
	{
		double map_x = map_waypoints_x[i];
		double map_y = map_waypoints_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

// Return the index of the next waypoint from position (x,y)
inline int CoordConverter::NextWaypoint(const double x, const double y, const double theta) const {
	int closestWaypoint = ClosestWaypoint(x,y);

	double map_x = map_waypoints_x[closestWaypoint];
	double map_y = map_waypoints_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if (angle > pi()/4.0)	{
		closestWaypoint++;
    closestWaypoint %= map_waypoints_x.size();
	}

	return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> CoordConverter::getSD(const double x, const double y, const double theta) const {
	int next_wp = NextWaypoint(x,y,theta);

	int prev_wp = next_wp-1;
	if (next_wp == 0)	{
		prev_wp  = map_waypoints_x.size()-1;
	}

	double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
	double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
	double x_x = x - map_waypoints_x[prev_wp];
	double x_y = y - map_waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000.0-map_waypoints_x[prev_wp];
	double center_y = 2000.0-map_waypoints_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = map_waypoints_s[prev_wp];
	frenet_s += distance(0.0,0.0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}



#endif
