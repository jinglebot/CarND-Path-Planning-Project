#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  // map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  // struct collider{

  //   bool collision ; // is there a collision?
  //   int  time; // time collision happens

  // };

  // int L = 1;

  int preferred_buffer = 0.5; // impacts "keep lane" behavior.

  int lane;

  double s;

  double v;

  double a;

  double target_speed = 49.5;


  string state;

  int id;

  double x;

  double y;

  double vx;

  double vy;

  double d;

  double yaw;

  double speed;

  // double DT = 0.02;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle (int lane, double s, double v);
  Vehicle (double x, double y, double s, double d, double yaw, double speed); // ego car
  Vehicle (double x, double y, double s, double d, double yaw, double speed, double a, int lane, string state); // ego car
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d, int lane);


  /**
  * Destructor
  */
  virtual ~Vehicle();

  double mph_to_mps(double v);

  vector<Vehicle> choose_next_state(map<int, Vehicle> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, Vehicle> predictions);

  vector<double> get_kinematics(map<int, Vehicle> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, Vehicle> predictions);

  // vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void prep_lane_change(Vehicle &vehicle, Vehicle &ego, double MAX_ACCEL, double MAX_DIST, double MAX_VEL);

  float position_at (float t);

  bool get_vehicle_behind(map<int, Vehicle> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle(map<int, Vehicle> &vehicles, Vehicle &ego, int lane, int prev_size, double DELTA_T, double MAX_DIST, Vehicle & rVehicle);

  Vehicle generate_predictions(int prev_size, double DT);

  void realize_next_state(vector<Vehicle> trajectory);

  // void configure(vector<int> road_data);

};

#endif