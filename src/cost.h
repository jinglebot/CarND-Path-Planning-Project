#ifndef COST_H
#define COST_H

#include <iostream>
#include <vector>
#include "vehicle.h"

using namespace std;


vector <double> lanes_speed_ahead(vector <double> &ave_vel, double ego_vel);

double lane_speed_ahead(double ave_vel, double ego_vel);


vector <double> lanes_speed_behind(vector <double> &ave_vel, double ego_vel);

double lane_speed_behind(double ave_vel, double ego_vel);


double ego_speed_cost(double ego_vel);

vector <double> calculate_COST (map<int, Vehicle> &vehicles, Vehicle &ego);


vector <double> lanes_distance(vector <double> &nearest_s, double ego_dist);

double lane_distance(double nearest_s, double ego_dist);


// double stay_in_lane_center(Vehicle &ego);


#endif