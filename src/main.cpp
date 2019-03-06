#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "helper.h"

using namespace std;


// for convenience
using json = nlohmann::json;

						
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

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

  // Initialize ego vehicle in lane 1, s starts at 0 m and initial vel is 0 mph)
  Vehicle ego = Vehicle(1, 0.0, 0, 6, MAX_ACCEL, "KL");
			
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road. vector < vector <double> >
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	// Define the actual (x,y) points we will use for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


// *******************************************
// START OF TODO LIST
// *******************************************
         
// *******************************************
// EGO VEHICLE
// *******************************************
			// if (prev_size > 0) 
			// {
			// 	ego.s = end_path_s;
   //          }

            int prev_size = previous_path_x.size();
			
			ego.s = car_s; 
			ego.d = car_d; 
			// ego.v = car_speed; 
			ego.lane = car_is_in_lane(car_d);
			
			// if (prev_size > 0) 
			// {
			// 	ego.s = end_path_s;
   //          }

            // bool too_close = false;
            // bool change_LANE = false;

// *******************************************
// NON - EGO VEHICLES
// *******************************************

			// place sensor_fusion vehicles in vehicles vector
			map<int, Vehicle> vehicles;
			for (int i = 0; i < sensor_fusion.size(); i++) 
			{
				int v_id = sensor_fusion[i][0];
            	double vx = sensor_fusion[i][3];
            	double vy = sensor_fusion[i][4];
            	double vehicle_v = get_v (vx, vy);

				double vehicle_s = sensor_fusion[i][5];
				double vehicle_d = sensor_fusion[i][6];
				int vehicle_lane = car_is_in_lane(vehicle_d);

				Vehicle vehicle = Vehicle(vehicle_lane, vehicle_s, mps_to_mph(vehicle_v), vehicle_d);
              	vehicles[v_id] = vehicle;
			}

			// Vehicles default
			// Vehicle vehicle = Vehicle(1, 500, MAX_VEL, 6);
			// vehicles[-1] = vehicle;

			// Vehicles visualization
			ego.see_vehicles(vehicles, ego);
			ego.transition_function(vehicles, ego);
			

			// Find and complete and optimal motion planning system.
			// Ego trajectory starts on finding out whether there is an obstacle ahead of it.
			// If there is, ego computes the costs of each possible trajectory: staying on the lane, changing to the left lane or to the right.
			// By comparing costs, ego decides which trajectory to take considering the state of the other vehicles in the environment.
			// Calculating costs only happens when there is a decision to make, else the vehicle stays on its lane when current state is optimal. 
            // sensor_fusion[0] id [1] x [2] y [3] vx [4] vy [5] s [6] d

			// Vehicle vehicle_ahead = Vehicle();
			// double cost_KEEP_lane = 0.00;
			// bool is_car_ahead = ego.get_vehicle(vehicles, ego, ego.lane, prev_size, DELTA_T, MAX_DIST, vehicle_ahead);
			// if (is_car_ahead) 
			// {
			// 	cost_KEEP_lane += calculate_COST (vehicle_ahead, ego, ego.lane, MAX_DIST, MAX_VEL);
			// 	// cout << "cost_KEEP_lane: " << cost_KEEP_lane << endl;
			// 	too_close = true;
   //          	change_LANE = true;
			// }

   //          // Change lanes
			// if (change_LANE) {
			// 	bool change_LEFT_lane = true;
			// 	bool change_RIGHT_lane = true;
			
			// 	double cost_LEFT_lane = 0.00;
			// 	double cost_RIGHT_lane = 0.00;

// *******************************************
// LANE CHANGING DECISION MAKING
// *******************************************
				
            	// if (cost_LEFT_lane > cost_RIGHT_lane) {
            	// 	if (cost_KEEP_lane > cost_RIGHT_lane) {
            	// 		ego.lane += 1;
						// cout << "CHANGE RIGHT LANE\n";
            		// } // else
            			// cout << "KEEP LANE\n";
            	// } else if (cost_KEEP_lane > cost_LEFT_lane) {
            	// 	ego.lane -= 1;
            		// cout << "CHANGE LEFT LANE\n";
            	// } // else
            		// cout << "KEEP LANE\n";
			// }

   //          if (too_close ) {
            	// decrement to around 5 m/s2
            // 	if (ego.v > vehicle_ahead.v)
           	// 		ego.v -= MAX_ACCEL;
           	// 	else
           	// 		ego.v = vehicle_ahead.v;
            // } 
            // else if (ego.v < MAX_VEL) {
            // 	ego.v += MAX_ACCEL;
            // }
            // if (ego.v < MAX_VEL) 
            // 	ego.v += ego.a;
            // else
            // 	ego.v -= ego.a;

// *******************************************
// TRAJECTORY
// *******************************************

         	// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          	vector <double> ptsx;
          	vector <double> ptsy;   

          	// reference x, yaw states
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          
          	// if previous size is almost empty, use the car as starting reference
       		if (prev_size < 2) {
              	// Use two points that make the path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
              
             	ptsx.push_back(prev_car_x);
             	ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
             	ptsy.push_back(car_y);
            }
          	//  use the previous path's end points as starting reference
			else {
           		// Redefine reference state as previous path endpoint
              	ref_x = previous_path_x[prev_size - 1];
           		ref_y = previous_path_y[prev_size - 1];

             	double ref_x_prev = previous_path_x[prev_size - 2];
              	double ref_y_prev = previous_path_y[prev_size - 2];
           		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
              
            	// Use two points that make the path tangent to the previous path's end points
             	ptsx.push_back(ref_x_prev);
             	ptsx.push_back(ref_x);
           
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }
         
         	// In Frenet, add evenly 30m spaced points ahead of the starting reference
            vector <double> next_wp0 = getXY(ego.s+30, 2+4*ego.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector <double> next_wp1 = getXY(ego.s+60, 2+4*ego.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector <double> next_wp2 = getXY(ego.s+90, 2+4*ego.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
          	ptsx.push_back(next_wp0[0]);
       		ptsx.push_back(next_wp1[0]);
       		ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);
          
          	// Shift Transformation
          	for (int i = 0; i < ptsx.size(); i++) {

              	// shift car reference angle to 0 degrees
              	double shift_x = ptsx[i] - ref_x;
             	double shift_y = ptsy[i] - ref_y;
              
              	ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              	ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

            }

          	// Create a spline
          	tk::spline s;
          
          	// set (x,y) points to the spline
          	s.set_points(ptsx,ptsy);
                    	
          	// Start with all of the previous path points from last time
            for (int i = 0; i < previous_path_x.size(); i++) {
              	  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
            }
          
          	// Calculate how to break up spline points so that we travel at our desired reference velocity
          	double target_x = 30.0; // in meters of spaced points
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          
          	double x_add_on = 0;
          
          	// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

              	double N = (target_dist/(0.02 * ego.v/2.24)); // convert to mps
              	// cout << "N: " << N << endl;
              	double x_point = x_add_on + (target_x)/N;
              	double y_point = s(x_point);
              
              	x_add_on = x_point;
              
              	double x_ref = x_point;
              	double y_ref = y_point;
              
              	// rotate back to normal after rotating it earlier
              	x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              	y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
              
              	x_point += ref_x;
              	y_point += ref_y;
              
              	next_x_vals.push_back(x_point);
              	next_y_vals.push_back(y_point);
            }
          
// *******************************************
// END OF TODO LIST
// *******************************************

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
