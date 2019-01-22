#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v) {

    this->lane = lane;
    this->s = s;
    this->v = v;
}

Vehicle::~Vehicle() {}

double Vehicle::mph_to_mps(double v) {
    return v * 0.44704;
}

bool Vehicle::get_vehicle(map<int, Vehicle> &vehicles, Vehicle &ego, int lane, int prev_size, double DELTA_T, double MAX_DIST, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    bool found_vehicle = false;
    int closest_car;
    double closest_car_dist = MAX_DIST;

    map<int, Vehicle>::iterator it = vehicles.begin();
    while(it != vehicles.end()) {
        int v_id = it->first;
        Vehicle possible_car_detected = it->second;
        if (possible_car_detected.lane == lane) {
           // if using previous points can project s value out
            possible_car_detected.s += ((double)prev_size * DELTA_T * mph_to_mps(possible_car_detected.v));
            if (((possible_car_detected.s > ego.s) && ((possible_car_detected.s - ego.s) < MAX_DIST)) || (lane != ego.lane && (fabs(possible_car_detected.s - ego.s) < MAX_DIST))) {
                found_vehicle = true;
                if ((possible_car_detected.s - ego.s) < closest_car_dist) {
                    closest_car = v_id;

                    // CHECK
                    // cout << "possible_car_detected: " << v_id << endl;
                    // cout << "ego.s: " << ego.s << endl;
                    closest_car_dist = possible_car_detected.s - ego.v;
                } 
            }
        }
        it++;
    }

    if (found_vehicle) {
        rVehicle = vehicles[closest_car];

        // CHECK
        // cout << "Car detected: " << closest_car << "\n"; // id of the car in front
        // cout << "Car detected.s: " << rVehicle.s << endl;
        // cout << "Car detected.v: " << rVehicle.v << endl;
    }

    return found_vehicle;
}

