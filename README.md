# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## In this project the goal is to design path planner to navigate around the virtual highway with other traffic car objects. The detailed information of the pipeline is presented below. 

## Details
The project includes a car traveling with traffic objects on a 3-lane highway. There are some restrictions on the maneuvers of the car;
* maximum speed
* maximum shock
* maximum acceleration
* collision avoidance

The car must decide its trajectory and speed during movement. In the `main.cpp` code, the trajectory of the vehicle is decided. In the output, the path planner generates (x, y) points that the car will visit every 0.02 seconds.

The steps to find these (x, y) points are as follows:
### Get ego vehicle location from main car localization data:

Initially, the car is started in lane 1, that is, in the center lane and at 0.0 speed. Then, with the lane width information, from the frenet coordinates (s,d) the lane of the car is found by the `car_lane ()` function. This lane information allows us to decide the location of other traffic vehicles relative to our lane.

### Get traffic vehicle information from sensor fusion data:

Sensor fuson data provides a lot of information about the other cars. The location of the other cars with respect to the ego vehicle must be found so that the next maneuver of the ego vehicle can be decided. The same `car_lane ()` function is used to find the lane of the traffic objects. Then, the other vehicles are checked to see if the ego is on the right or left of the vehicle or in the same lane. Three flags(traffic_car_ahead, traffic_car_left, and traffic_car_right) are set according to this lane comparison and the relative distance.

### Make a decision :

In the decision maker part, it checks for the traffic_car_ahead flag. If it is true that it is not empty, then it will make the decision to turn left, turn right, or reduce the speed, depending on the availability of other lane. 

If the ahead of the vehicle is empty and velocity is not same as maximum velocity, then it will increase the speed by small difference.

### Find trajectory points

 After decision, the last two points of the previous trajectory are used in conjunction with three points at a far distance (30, 60, 90 meters) according to the decided lane. The spline is calculated based on those points. The coordinates are transformed to local car coordinates.

All criteria in this rubic point is met as follows:

* The car drives within the speed limit
* Max acceleration and jerk are not exceeded
* No collisions
* The car keeps the current lane till it finds an opportunity to make lane change and reach higher speed
* The car is able to change lanes without exceeding max acceleration and jerk

<p align="center">
  <img width="800" height="400" src="./data/pp_carnd.png ">
</p>

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 



## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}


