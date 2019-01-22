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

  /**
  * Constructor
  */
  Vehicle();
  Vehicle (int lane, double s, double v);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  double mph_to_mps(double v);

  bool get_vehicle(map<int, Vehicle> &vehicles, Vehicle &ego, int lane, int prev_size, double DELTA_T, double MAX_DIST, Vehicle & rVehicle);

};

#endif