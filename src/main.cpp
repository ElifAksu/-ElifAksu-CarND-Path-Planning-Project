#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
int car_lane(double d) {
    int lane_id = -1;
    if (d > 0 && d < 4.0) {
        lane_id = 0;
    } else if (d > 4.0 && d < 8.0) {
        lane_id = 1;
    } else if (d > 8.0 && d < 12.0) {
        lane_id = 2;
    }
    return lane_id;
}
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

 
    int car_lane_id = 1;  
    double ref_vel = 0.0; 
   h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, 		  &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &car_lane_id]
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

          

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
	  if (previous_path_x.size() > 0)
          {
            car_s = end_path_s;
          }
          car_lane_id=car_lane(car_d);
          int traffic_car_ahead =0; //empty
          int traffic_car_right =0; //empty
          int traffic_car_left =0;  //empty
          int collusion =0;  //avoidance
           for (int i=0; i<sensor_fusion.size(); ++i)
          {
            
            
				    auto traffic_car_id = sensor_fusion [i][0];
				    double traffic_car_x = sensor_fusion [i][1];
				    double traffic_car_y = sensor_fusion [i][2];
				    double traffic_car_vx = sensor_fusion [i][3];
				    double traffic_car_vy = sensor_fusion [i][4];
				    double traffic_car_s = sensor_fusion [i][5];
				    double traffic_car_d = sensor_fusion [i][6];

            int traffic_lane_id=car_lane(traffic_car_d);
            double traffic_car_speed = sqrt(traffic_car_vx * traffic_car_vx + traffic_car_vy * traffic_car_vy);
            traffic_car_s += ((double)previous_path_x.size()*0.02*traffic_car_speed); //turning max acceleraion
            //where is the traffic car
            double range_max = car_s+30;
            double range_min = car_s-30;
          if (car_lane_id == traffic_lane_id)
            {
              std::cout << "car ahead distance: " << traffic_car_s-car_s <<std::endl;
              std::cout << "traffic_car_speed: " << traffic_car_speed <<std::endl;
              if((traffic_car_s > car_s) && (traffic_car_s<range_max))
              {traffic_car_ahead =1 ;
              if((traffic_car_s > car_s) && (traffic_car_s<(range_max-15)) || (ref_vel>traffic_car_speed))
              { collusion=1;}
              
              }
            }
            else if (traffic_lane_id - car_lane_id == 1)
            {
              if((traffic_car_s > range_min) && (traffic_car_s<range_max))
              {traffic_car_right =1 ;}
            }
            else if (traffic_lane_id - car_lane_id == -1)
            {
              if((traffic_car_s > range_min) && (traffic_car_s<range_max))
              {traffic_car_left =1 ;}
            }

          }



            double speed_diff = 0;
            const double MAX_speed = 49.5;
            const double MAX_acc = 0.224;
            if ( traffic_car_ahead ) { // ahead is not empty
              if ( !traffic_car_left && car_lane_id > 0 ) {  // there is a left lane and empty  
                car_lane_id--; // Change lane left
              } else if ( !traffic_car_right && car_lane_id != 2 ){ // there is a right lane and empty 
                car_lane_id++; // Change lane right.
              } 
              
              else { //lanes are not empty
                speed_diff -= MAX_acc;
              
              }
            } 
            else { 
               //ahead is empty
                if (car_lane_id != 1)
                  { // if ego car is not on the center lane.
                    if ((car_lane_id == 0 && !traffic_car_right) || (car_lane_id == 2 && !traffic_car_left))
                    {
                      car_lane_id = 1; // center is good
                    }
                  }
                if (ref_vel < MAX_speed ) {
                speed_diff += MAX_acc;
              }
            }

                    
                    std::cout<< "Sensor element size : "<<sensor_fusion.size()<<std::endl;
                    std::cout << "mylane: " << car_lane_id <<std::endl;
                    
                    std::cout << "car ahead: " << traffic_car_ahead << std::endl;
                    std::cout << "car right: " << traffic_car_right << std::endl;
                    std::cout << "car left : " << traffic_car_left << std::endl;
                    std::cout << "speed_diff: " << speed_diff << std::endl;
                    std::cout << "ref_vel: " << ref_vel << std::endl;

                    
          //define a path made up of (x,y) points that the car will visit
          vector<double> next_x_vals;
          vector<double> next_y_vals;

         //pts will sent to the next_x,y_vals
          vector<double> pts_x;
          vector<double> pts_y;

        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);       

         //pts starting points are get from prev path info
          if (previous_path_x.size() < 2)
          {
            
            double car_x_prev = car_x - cos(car_yaw);
            double car_y_prev = car_y - sin(car_yaw);

            pts_x.push_back(car_x_prev);
            pts_x.push_back(car_x);
            pts_y.push_back(car_y_prev);
            pts_y.push_back(car_y);
          }
          
          else
          {
            
           ref_x = previous_path_x[previous_path_x.size()-1];
           ref_y = previous_path_y[previous_path_x.size()-1];

            double ref_x_prev = previous_path_x[previous_path_x.size()-2];
            double ref_y_prev = previous_path_y[previous_path_x.size()-2];

           ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            
            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * car_lane_id, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * car_lane_id, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * car_lane_id, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          // Making coordinates to local car coordinates.
          for (int i = 0; i < pts_x.size(); i++) {
                        double shift_x = pts_x[i] - ref_x;
                        double shift_y = pts_y[i] - ref_y;

                        pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                        pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

                    // Create the spline.
          tk::spline spline_;
          spline_.set_points(pts_x, pts_y); // 2 prev points and 3 future points -spline eqn
          for(int i = 0; i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x = 30.0;
          double target_y = spline_(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;
      

          for (int i=1; i<=50-previous_path_x.size(); ++i)
          {
            ref_vel += speed_diff;
            if (ref_vel > MAX_speed)
            {
              ref_vel = MAX_speed;
            }
            else if (ref_vel < 0.224)
            {
              ref_vel = 0.224;
            }
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = spline_(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

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