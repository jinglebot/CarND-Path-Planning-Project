#ifndef HELPER_H
#define HELPER_H

#include <iostream>
#include <vector>

using namespace std;


// At each timestep, ego can set acceleration to value between 
extern const double MAX_ACCEL; // == 5 m/s^2
// double DELTA_T = 0.02; // seconds
extern const int NUM_LANES;

extern const double MAX_DIST; // meters

// MAX_VEL is the acceptable vel, not too slow yet not too fast to go over the SPEED_LIMIT
extern const double MAX_VEL; // mph

// SPEED_LIMIT is legal speed allowed
extern const double SPEED_LIMIT;

extern const double STOP_COST;

extern const double DIST_BUFFER;

extern const double D_TIME;

extern const double MIN_TIME_LANE_CHANGE;

// For converting back and forth between radians and degrees.
constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

int car_is_in_lane(int d);

double get_v (double vx, double vy);

double mps_to_mph(double v); 

double mph_to_mps(double v);


#endif
