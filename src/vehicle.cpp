#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "classifier.h"
// #include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v) {

    this->lane = lane;
    this->s = s;
    this->v = v;
}

Vehicle::Vehicle (double x, double y, double s, double d, double yaw, double v, double a, int lane, string state) {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->v = v;
    this->lane = lane;
    this->a = a; 
    this->state = state;   
}
            

Vehicle::Vehicle (int id, double x, double y, double vx, double vy, double s, double d, int lane) {
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->lane = lane;
}


Vehicle::~Vehicle() {}

double Vehicle::mph_to_mps(double v) {
    return v * 0.44704;
}

// void Vehicle::choose_next_state() {
vector<Vehicle> Vehicle::choose_next_state(map<int, Vehicle> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    // cout << "In choose_next_state function" << endl;
    vector <string> states = successor_states();
    // cout << "states size: " << states.size() << endl;
            // float cost;
    // vector<float> costs;
    vector <vector <Vehicle> > final_trajectories;
   
    for (vector <string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
// //             cost = calculate_cost(*this, predictions, trajectory);
// //             costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }
    // cout << endl;

//     vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
//     int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[0];
    // return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    // cout << "In successor_states function\n";
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        // if (lane != lanes_available - 1) {
        //     states.push_back("PLCR");
        //     states.push_back("LCR");
        // }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    // cout << "State: " << state << endl;
    // for (int i = 0; i < states.size(); i++) {
    //     cout << "successor_state: " << i+1 << ": " << states[i] << "\n";
    // }
    // cout << endl;
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, Vehicle> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    // cout << "In generate_trajectory function\n";
    vector <Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        // cout << " to change to CS" << endl;
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        // cout << " to change to KL" << endl;
        trajectory = keep_lane_trajectory(predictions);
    } 
        // else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    // //     trajectory = lane_change_trajectory(state, predictions);
    //     cout << " to change to LCL/LCR" << endl;
    // } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    // //     trajectory = prep_lane_change_trajectory(state, predictions);
    //     cout << " to change to PLCL/PLCR" << endl;
    // }
    return trajectory;
}

vector<double> Vehicle::get_kinematics (map < int, Vehicle > predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    If there is a car ahead, ego must match the v of car ahead and travel at the same speed of traffic.
    If there is a car behind, ego must travel at minimum v of car ahead, speed limit or max acceleration.
    If there is no car ahead, ego can travel at max v and acceleration.
    */
    // double max_velocity_accel_limit = this->MAX_ACCELERATION + this->v;
    // cout << "V: " << this->v << endl;
    // cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << endl;
    // cout << "max_velocity_accel_limit: " << max_velocity_accel_limit << endl;
    double new_position;
    double new_velocity = this->v;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (this->v < target_speed) {
        // new_velocity =  this->v + this->TOTAL_ACCELERATION;
        // cout << "accelerated_velocity: " << new_velocity << endl;
        // new_accel = this->TOTAL_ACCELERATION;
        // max_velocity_accel_limit = this->TOTAL_ACCELERATION + this->v;
        // cout << "max_velocity_accel_limit: " << max_velocity_accel_limit << endl;
        // new_velocity = max_velocity_accel_limit;
        // cout << "new_velocity: " << new_velocity << endl;
        // cout << "TOTAL_ACCELERATION: " << TOTAL_ACCELERATION << endl;  
    } else {
        // new_velocity = this->v - this->TOTAL_ACCELERATION;
        // cout << "reduced_velocity: " << new_velocity << endl;   
        // new_accel = - this->TOTAL_ACCELERATION;
    }

    

    // if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        // cout << "Car present ahead\n";
        // cout << "S of ego car: " << this->s << "\n";
        // cout << "S of vehicle_ahead: " << vehicle_ahead.s << "\n";
        // cout << "S difference: " << abs(vehicle_ahead.s - this->s) << "\n";
        // cout << "V of ego car: " << this->v << "\n";
        // cout << "V of vehicle_ahead: " << vehicle_ahead.v << "\n";
        
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            // cout << "Car present behind\n";
            // cout << "S of ego car: " << this->s << "\n";
            // cout << "S of vehicle_behind: " << vehicle_behind.s << "\n";
            // cout << "S difference: " << abs(vehicle_behind.s - this->s) << "\n";
            // new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
            // cout << "new_velocity: " << new_velocity << "\n";

        } else {
            // cout << "No car present behind\n";
            // float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            // cout << "Vehicle ahead s: " << vehicle_ahead.s << "\n";
            // cout << "S of ego car: " << this->s << "\n";
            // cout << "A of ego car: " << this->a << "\n";
            // cout << "max_velocity_in_front: " << max_velocity_in_front << "\n";
            // cout << "max_velocity_accel_limit: " << max_velocity_accel_limit << "\n";
            // cout << "this->target_speed: " << this->target_speed << "\n";
        
            // new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
            // cout << "new_velocity: " << new_velocity << "\n";
        
        }

    // } else {
         // cout << "No car present ahead\n";
         // cout << "V of ego car: " << this->v << "\n";
         // cout << "max_velocity_accel_limit: " << max_velocity_accel_limit << "\n";
         // cout << "this->target_speed: " << this->target_speed << "\n";
         // new_velocity = min(max_velocity_accel_limit, this->target_speed);
         // cout << "new_velocity: " << new_velocity << "\n";
       
    // }
    // cout << "new_velocity: " << new_velocity << "\n";
    // cout << "new_velocity - this->v: " << new_velocity - this->v << "\n";
    // cout << "DT: " << DT << "\n";
    
    // if ((new_velocity - this->v) > 0) {
        // new_accel = (new_velocity - this->v) / DT; //Equation: (v_1 - v_0)/t = acceleration
    new_accel = this->a;
    // }
    // else {
        // new_accel = 0;
    // }
    new_position = this->s ;
    // new_position = this->s + new_velocity + new_accel / 2.0;
    // new_position = this->s + new_velocity * DT + 0.5 * new_accel * DT * DT;
    // new_velocity = this->v;
    // new_accel = this->a;
    // cout << "new_position: " << new_position << "\t";
    // cout << "new_velocity: " << new_velocity << "\t";
    // cout << "new_accel: " << new_accel << "\n";
    return{new_position, new_velocity, new_accel};
    
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    // cout << "In constant_speed_trajectory function" << endl;

    // should horizon == 1 or == t? should horizon x 0.02?

    // float next_pos = position_at(DT);

    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v)};
    // vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v), 
                                  // Vehicle(this->lane, next_pos, this->v)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, Vehicle> predictions) {
    /*
    Generate a keep lane trajectory.
    KLT means stay in your lane. If there is a vehicle in front or behind, match traffic speed.
    */
    // cout << "In keep_lane_trajectory function" << endl;
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v)};
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    cout << "new_s: " << new_s << endl;    
    double new_v = kinematics[1];
    cout << "new_v: " << new_v << endl;    
    double new_a = kinematics[2];
    cout << "new_a: " << new_a << endl;    
    trajectory.push_back(Vehicle(this->lane, new_s, new_v));
    return trajectory;
}
                            
void Vehicle::prep_lane_change (Vehicle &vehicle, Vehicle &ego, double MAX_ACCEL, double MAX_DIST, double MAX_VEL) {
    /*
    Generate a trajectory preparing for a lane change.
    you need lane, lane v, cars in that lane, cost
    */      
    // bool too_close = (vehicle.s - ego.s) < MAX_DIST;
    //         if (too_close ) {
                // decrement to around 5 m/s2
                
            // if (ego.v < vehicle.v)
            //     ego.v = vehicle.v;
            // else
            
    // ego.v = vehicle.v;
//     float new_s;
//     float new_v;
//     float new_a;
//     Vehicle vehicle_behind;
//     int new_lane = this->lane + lane_direction[state];
//     vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
//     vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

//     if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
//         //Keep speed of current lane so as not to collide with car behind.
//         new_s = curr_lane_new_kinematics[0];
//         new_v = curr_lane_new_kinematics[1];
//         new_a = curr_lane_new_kinematics[2];
        
//     } else {
//         vector<float> best_kinematics;
//         vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
//         //Choose kinematics with lowest velocity.
//         if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
//             best_kinematics = next_lane_new_kinematics;
//         } else {
//             best_kinematics = curr_lane_new_kinematics;
//         }
//         new_s = best_kinematics[0];
//         new_v = best_kinematics[1];
//         new_a = best_kinematics[2];
//     }

//     trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
//     return trajectory;
// }

// vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
//     /*
//     Generate a lane change trajectory.
//     */
//     int new_lane = this->lane + lane_direction[state];
//     vector<Vehicle> trajectory;
//     Vehicle next_lane_vehicle;
//     //Check if a lane change is possible (check if another vehicle occupies that spot).
//     for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
//         next_lane_vehicle = it->second[0];
//         if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
//             //If lane change is not possible, return empty trajectory.
//             return trajectory;
//         }
//     }
//     trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
//     vector<float> kinematics = get_kinematics(predictions, new_lane);
//     trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
//     return trajectory;
// }

// void Vehicle::increment(int dt = 1) {
// 	this->s = position_at(dt);
// }

// float Vehicle::position_at (float t) {
//     // cout << "s: " << this->s << endl;
//     // cout << "v: " << this->v * t * 1.0 << endl;
//     // cout << "a: " << this->a * t * t / (2.0 * 3600) << endl;
//     // cout << "t: " << t << endl;

//     return this->s + this->v * t;
}

bool Vehicle::get_vehicle_behind(map<int, Vehicle> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    float max_s = -1;
    bool found_vehicle = false;
    // Vehicle temp_vehicle;
    // for (map<int, Vehicle>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    //     temp_vehicle = it->second;
    //     if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
    //         max_s = temp_vehicle.s;
    //         // cout << "Car " << it->first << endl;
    //         rVehicle = temp_vehicle;
    //         found_vehicle = true;
    //     }
    // }
    return found_vehicle;
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
        // cout << "Car detected: " << closest_car << "\n"; // id of the car in front
        // cout << "Car detected.s: " << rVehicle.s << endl;
        // cout << "Car detected.v: " << rVehicle.v << endl;
    }

            // double check_speed_x;
            // double check_car_s;
            // // for (int i = 0; i < sensor_fusion.size(); i++) {
                // car is in my lane
                // float d = sensor_fusion[i][6]; // d
                // if (d < (2 + 4 * ego.lane + 2) && d > (2 + 4 * ego.lane - 2) ) {
                    // int v_id = sensor_fusion[i][0];
                    // double vx = sensor_fusion[i][3];
                    // double vy = sensor_fusion[i][4];
                    // check_speed_x = sqrt(vx * vx + vy * vy);
                    // check_car_s = sensor_fusion[i][5];
                    // if (ego.v > check_speed) is_faster = true;
                    // cout << "check_speed: " << check_speed << endl;

                    // check_car_s += ((double)prev_size * 0.02 * check_speed_x); // if using previous points can project s value out
                    // check s values greater than ego and s gap
                    // if ((check_car_s > car_s) && ((check_car_s - car_s) < MAX_DIST) ) {
                        // too_close = true;
                        // change_LANE = true;
                    //     cout << "Car in front: " << v_id << "\t"; // speed of the car in front
                    //     cout << "Check_car_s: " << check_car_s << "\t"; // speed of the car in front
                    // }   cout << "Check speed x: " << check_speed_x << endl; // speed of the car in front
                        
            //     }
            // }




    // float min_s = 0;
    // bool found_vehicle = false;
    // Vehicle temp_vehicle;
    // for (map<int, Vehicle>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    //     temp_vehicle = it->second;
    //     // if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
    //     if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s /* && abs(temp_vehicle.s - this->s) < min_s */) {
    //         min_s = temp_vehicle.s;
    //         // cout << "Car " << it->first << endl;
    //         rVehicle = temp_vehicle;
    //         found_vehicle = true;
    //     }
    // }
    return found_vehicle;
}

// Vehicle Vehicle::generate_predictions(int prev_size, double DELTA_T) {
//     /*
//     Generates predictions for non-ego vehicles to be used
//     in trajectory generation for the ego vehicle.
//     */
//     // cout << "In generate_predictions function" << endl;
// 	// vector<Vehicle> predictions;
//  //    for(int i = 0; i < horizon; i++) {
//       // cout << "horizon " << i << endl;
//       // cout << "curn_s: " << this->s << endl;
//       // cout << "curn_v: " << this->v << endl;
//       double next_s = prev_size * DELTA_T * this->v;
//       // cout << "next_s: " << next_s << endl;
//       // float next_v = this->v;
//       // if (i < horizon-1) {
//       //   float t = (i + 1) * 0.02;
//       //   next_v = (position_at(t) - s) / t;
//       // }
//     return Vehicle(this->lane, next_s, this->v);

// }

// void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
//     /*
//     Sets state and kinematics for ego vehicle using the last state of the trajectory.
//     */
//     Vehicle next_state = trajectory[1];
//     this->state = next_state.state;
//     this->lane = next_state.lane;
//     this->s = next_state.s;
//     this->v = next_state.v;
//     // this->a = next_state.a;
// }

// void Vehicle::configure(vector<int> road_data) {
//     /*
//     Called by simulator before simulation begins. Sets various
//     parameters which will impact the ego vehicle. 
//     */
//     target_speed = road_data[0];
//     lanes_available = road_data[1];
//     goal_s = road_data[2];
//     goal_lane = road_data[3];
//     max_acceleration = road_data[4];
// }