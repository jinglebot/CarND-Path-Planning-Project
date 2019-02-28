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

  int lane;

  double s;

  double v;

  double d;

  double a;

  string state;

  vector <int> lane_nearest_cars_ahead;

  vector <int> lane_nearest_cars_behind;

  map<string, vector<string> > possible_states;



  /**
  * Constructor
  */
  Vehicle();
  Vehicle (int lane, double s, double v, double d);
  Vehicle(int lane, double s, double v, double d, double a, string state);


  /**
  * Destructor
  */
  virtual ~Vehicle();


  void see_vehicles(map<int, Vehicle> &vehicles, Vehicle &ego);
 
  void see_cars(map<int, Vehicle> &vehicles, map<int, Vehicle> &cars);


  void transition_function(map<int, Vehicle> &vehicles, Vehicle &ego);

  void get_sensor_config(map<int, Vehicle> &vehicles, Vehicle &ego);


  vector <int> lane_nearest_cars(map<int, Vehicle> &vehicles, map<int, Vehicle> &cars, Vehicle &ego);

  void get_possible_states();

  string get_next_state(map<int, Vehicle> &vehicles, Vehicle &ego);


  void implement_trajectory(map<int, Vehicle> &vehicles, Vehicle &ego, string state);

  void implement_trajectory_KL(map<int, Vehicle> &vehicles, Vehicle &ego);
  
  void implement_trajectory_PCLL(map<int, Vehicle> &vehicles, Vehicle &ego);
  
  void implement_trajectory_CLL(map<int, Vehicle> &vehicles, Vehicle &ego);
  
  void implement_trajectory_PCLR(map<int, Vehicle> &vehicles, Vehicle &ego);
  
  void implement_trajectory_CLR(map<int, Vehicle> &vehicles, Vehicle &ego);

};

#endif
