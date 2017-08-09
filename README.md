# Udacity CarND-Path-Planning-Project
   
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We receive from the simulator the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway.
 The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 ms^-2^ and jerk that is greater than 50 ms^-3^.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

You will need as well the simulator that can be found [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) 


## Project implementation overview

To implement a solution to this project i have splitted the process in 5 differents sub-process that will be discussed in detail in the remaining of the document.

* Interpolate the waypoints so we can generate smooth path
* Update and predict other vehicules possible trajectory
* Use a portion of the previous path as starting point of the new one
* Define the behaviour to adopt like changing lane
* Based on behaviour, define a trajectory

#### Waypoints interpolation

In order to be able to go from and to the two coordinate systems used (X,Y globalmap and Frenet S,D) we had to interpolate the waypoints provided. If we use the raw data we get really sharp direction change during the path generation.
To interpolate the waypoints, i created a class `CoordConverter` that use the waypoints file provided and a spline interpolation on X,Y,dx, dy relative to s.
So to get the X coordinate from S,D : X = spline_X(S) + spline_dx(S) * D

#### Update other vehicules state

To track the other vehicules state, i created a class `Vehicle` that use the data from the sensor fusion to update the new state of the vehicle.
This class provide a function to get the state at a time t. There is a big assumption here as we assume a constant velocity. So t should not be that big. I didn't implement more complex prediction model like multi-model algorithm or data driven algorithm.

#### Use previous path

In order to make the drive as smooth as possible we need to re-use data point from the previous path. As durint our computation of a new path the car is still moving some points will be already used. This will help as well if we generate a big latency. But we have to keep it low as well if we want to be able to react to our environment change. So i decided to keep only 10 data points which correspond to a 0.2 seconds of reaction time. 
The next point in the previous path is used as starting point for our future path generation.

#### Behaviour planning

The behaviour process implemented is looking only at 3 possible behaviours :

* Change to Left lane
* Keep current lane
* Change to Right lane

The process is using a cost minimizing algorithm.
The cost is using the following factors

* If there is a car in Front in the lane, use the speed and distance to get a cost. 
	Speed Cost = (MAX_SPEED - target_speed) / MAX_SPEED
	Dist Cost = target_distance / MIN_BUFFER_BETWEEN_CAR

* If there is a car Behind in the lane and not enough buffer, bump the cost.

* Use the distance to travel to that lane from our current lane
	change lane cost = (target_d - my_d) / (1.5 * LANE_WIDTH)

After getting the cost of all 3 options the lowest one is selected.

#### Generating a new path

Based on the lane selected by the behaviour planning, look to see if there is any car in front in that lane to use it to define our end state.
If there is a car and close to us, try to keep a mininum buffer. If no car or too far, try to reach max speed.

Once we have our end state we can generate a jerk minimize trajectory using a quintic polynomial.
We now using a loop to potentialy update the trajectory if we may have a collision or speeding.

* We update the normal speed until we have no more collision
* We reduce the acceleration until we don't exceed the speed limit

Then we convert the trajectory from frenet coordinate to X,Y global map coordinate.

## Result

Here is a [video](https://youtu.be/r0tpHM_FxnM)  where we can see the car navigating smoothly around.

