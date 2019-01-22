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


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) 
{

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

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// *******************************************
// LANE CHANGE
// *******************************************

int car_is_in_lane(int d) {

 	if (d < 4 && d > 0 ) {
		return 0;
	} else if (d < 8 && d >= 4 ) {
		return 1;
	} else if (d < 12 && d >= 8 ) {
		return 2;
	}
	return 0;

}

double get_v (double vx, double vy) 
{

	return sqrt(vx * vx + vy * vy);

}

double mph_to_mps(double v) 
{

	return v * 0.44704;

}

// *******************************************
// COST FUNCTION
// *******************************************


double calculate_COST (Vehicle &possible_car_detected, Vehicle &ego, int new_LANE, double MAX_DIST, double MAX_VEL) 
{

    double stop_cost = 10.00;
    double cost = 0.00;

    // if slow vehicle is ahead of lane change gap
    if ((possible_car_detected.s > ego.s) && ((possible_car_detected.s - ego.s) < MAX_DIST)) 
    {

    	// if it's fast, there's a low cost
	    // if it's slow, there's a great cost
         if (possible_car_detected.v < ego.v) 
        {
        
            cost += stop_cost * (ego.v - possible_car_detected.v) / possible_car_detected.v;                   
        
        }
    }

    // if vehicle is within lane change gap
    if ((fabs(possible_car_detected.s - ego.s) <= (MAX_DIST * 2)) && (new_LANE != ego.lane)) 
    {
    

        cost += stop_cost;
    
    }

    // if fast vehicle is behind lane change gap
 	if ((possible_car_detected.s < ego.s) && ((ego.s - possible_car_detected.s) < (MAX_DIST * 2))) 
	{
       
       if (possible_car_detected.v > ego.v) 
       {
       
            cost += stop_cost * (possible_car_detected.v - ego.v) / ego.v;                          	
       
       }

	}
    
    // cost == 0 if there is no car in new lane or if car ahead in new lane is fast or car behind in new lane is slow

	return cost;
}
						
// *******************************************

// At each timestep, ego can set acceleration to value between 
double MAX_ACCEL = 0.224; // == 5 m/s^2
double MAX_VEL = 49.75; // mph
double DELTA_T = 0.02; // seconds
double MAX_DIST = 50; // meters

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
  Vehicle ego = Vehicle(1, 0.0, 0.0);
			
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
         
            int prev_size = previous_path_x.size();
			
			ego.s = car_s; 
			
			if (prev_size > 0) 
			{
				ego.s = end_path_s;
            }

			ego.lane = car_is_in_lane(car_d);
			
			// CHECK
			// cout << "Ego car\t";
			// cout << "Lane: " << ego.lane << "\t";
			// cout << "S: " << ego.s << "\t";
			// cout << "V: " << ego.v << "\n";

            bool too_close = false;
            bool change_LANE = false;

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

	            // CHECK
	            // cout << "Car " << v_id << ":\t";
				// cout << "Lane: " << vehicle_lane << "\t";
				// cout << "S: " << vehicle_s << "\t";
				// cout << "V " << vehicle_v << "\n";

				Vehicle vehicle = Vehicle(vehicle_lane, vehicle_s, vehicle_v);
              	vehicles[v_id] = vehicle;
			}

			// Future s for non-ego vehicles
			// map<int,Vehicle > predictions;
			// map<int, Vehicle>::iterator it = vehicles.begin();
			// while(it != vehicles.end()) {
		    //    int v_id = it->first;

		    //    Vehicle preds = it->second.generate_predictions(prev_size, DELTA_T);
            //	  predictions[v_id] = preds;
		    // }

// *******************************************
// EGO VEHICLE
// *******************************************

			// Ego trajectory starts on finding out whether there is an obstacle ahead of it.
			// If there is, ego computes the costs of each possible trajectory: staying on the lane, changing to the left lane or to the right.
			// By comparing costs, ego decides which trajectory to take considering the state of the other vehicles in the environment.
			// Calculating costs only happens when there is a decision to make, else the vehicle stays on its lane when current state is optimal. 
            // sensor_fusion[0] id [1] x [2] y [3] vx [4] vy [5] s [6] d

			Vehicle vehicle_ahead = Vehicle();
			double cost_KEEP_lane = 0.00;
			bool is_car_ahead = ego.get_vehicle(vehicles, ego, ego.lane, prev_size, DELTA_T, MAX_DIST, vehicle_ahead);
			if (is_car_ahead) 
			{
				cost_KEEP_lane += calculate_COST (vehicle_ahead, ego, ego.lane, MAX_DIST, MAX_VEL);
				cout << "cost_KEEP_lane: " << cost_KEEP_lane << endl;
				too_close = true;
            	change_LANE = true;
			}

            // Change lanes
			if (change_LANE) {
				bool change_LEFT_lane = true;
				bool change_RIGHT_lane = true;
			
				double cost_LEFT_lane = 0.00;
				double cost_RIGHT_lane = 0.00;

// *******************************************
// LEFT LANE CHANGING
// *******************************************
				
				// Situation 1: If lane is already the innermost lane, no changing of lanes will occur.
				// Situation 2: If there is a vehicle within the lane changing space of 30m ahead as well as behind the ego car, no changing of lanes can occur.
				// Situation 3: If there is a vehicle ahead of the lane changing space and faster than ego car, changing of lanes can occur.
				// Situation 4: If there is a vehicle behind the lane changing space and faster than ego car, changing of lanes cost will be evaluated.
				// Situation 5: If there is a vehicle ahead of the lane changing space and slower than ego car, changing of lanes cost will be evaluated.
				// Situation 6: If there is a vehicle behind the lane changing space and slower than ego car, changing of lanes can occur.
				// Situation 7: If there is no vehicle in the new lane, changing of lanes cost will occur.
				// Situation 8: Default ego car situation, no changing of lanes will occur.

				int new_LANE = ego.lane - 1;
				
				if ( new_LANE < 0) {
					change_LEFT_lane = false;
					cost_LEFT_lane += 100;
				} else {
				    map<int, Vehicle>::iterator it = vehicles.begin();
				    while(it != vehicles.end()) {
				        int v_id = it->first;
				        Vehicle possible_car_detected = it->second;
				        if (possible_car_detected.lane == new_LANE) {

				            // if using previous points can project s value out
				            possible_car_detected.s += ((double)prev_size * DELTA_T * mph_to_mps(possible_car_detected.v));

				            cost_LEFT_lane += calculate_COST (possible_car_detected, ego, new_LANE, MAX_DIST, MAX_VEL);
        				}
        				it++;
    				}
    			}
				cout << "cost_LEFT_lane: " << cost_LEFT_lane << endl;

// *******************************************
// RIGHT LANE CHANGING
// *******************************************
				
				new_LANE = ego.lane + 1;
				Vehicle vehicle_on_right = Vehicle();
				if (new_LANE > 2) {
					change_RIGHT_lane = false;
					cost_RIGHT_lane += 100;
				} else {
					bool is_car_on_right = ego.get_vehicle(vehicles, ego, new_LANE, prev_size, DELTA_T, MAX_DIST, vehicle_on_right);
			
				    map<int, Vehicle>::iterator it = vehicles.begin();
				    while(it != vehicles.end()) {
				        int v_id = it->first;
				        Vehicle possible_car_detected = it->second;
				        if (possible_car_detected.lane == new_LANE) {

				            // if using previous points can project s value out
				            possible_car_detected.s += ((double)prev_size * DELTA_T * mph_to_mps(possible_car_detected.v));

				            cost_RIGHT_lane += calculate_COST (possible_car_detected, ego, new_LANE, MAX_DIST, MAX_VEL);
        				}
        				it++;
    				}
				}
				cout << "cost_RIGHT_lane: " << cost_RIGHT_lane << endl;

// *******************************************
// LANE CHANGING DECISION MAKING
// *******************************************
				
            	if (cost_LEFT_lane > cost_RIGHT_lane) {
            		if (cost_KEEP_lane > cost_RIGHT_lane) {
            			ego.lane += 1;
						cout << "CHANGE RIGHT LANE\n";
            		} else
            			cout << "KEEP LANE\n";
            	} else if (cost_KEEP_lane > cost_LEFT_lane) {
            		ego.lane -= 1;
            		cout << "CHANGE LEFT LANE\n";
            	} else
            		cout << "KEEP LANE\n";
			}

            if (too_close ) {
            	// decrement to around 5 m/s2
            	if (ego.v > vehicle_ahead.v)
           			ego.v -= MAX_ACCEL;
           		else
           			ego.v = vehicle_ahead.v;
            } 
            else if (ego.v < MAX_VEL) {
            	ego.v += MAX_ACCEL;
            }

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
            // return {next_x_vals, next_y_vals};
            // cout << "previous_path_x size: " << previous_path_x.size() << endl;
          
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
