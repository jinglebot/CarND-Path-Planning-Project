#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "helper.h"

using namespace std;


/**
 * Initializes Vehicle
 */


Vehicle::Vehicle(){}


Vehicle::Vehicle(int lane, double s, double v, double d) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->d = d;
}


Vehicle::Vehicle(int lane, double s, double v, double d, double a, string state) {

    this->a = a;    
    this->state = state;
}


Vehicle::~Vehicle() {}


void Vehicle::see_vehicles(map<int, Vehicle> &vehicles, Vehicle &ego) {
    // Find vehicles ahead of ego car
    map <int, Vehicle> cars_ahead;
    map <int, Vehicle> cars_behind;
    map<int, Vehicle>::iterator it;
    for (it = vehicles.begin(); it != vehicles.end(); it++) {
        int v_id = it->first;
        Vehicle car = it->second;

        // Separate cars ahead and cars behind
        if (car.s > ego.s) {
            cars_ahead[v_id] = car;
        } else {
            cars_behind[v_id] = car;
        }
    }

    see_cars(vehicles, cars_ahead);
    cout << "=========================================" << "Ego Car S: " << ego.s << "\t==========================================================\n";
    see_cars(vehicles, cars_behind);
}


void Vehicle::see_cars(map<int, Vehicle> &vehicles, map<int, Vehicle> &cars) {

    // List cars ahead/behind of ego by pos
    map <double, int> cars_order;
    map<int, Vehicle>::iterator it;
    for (it = cars.begin(); it != cars.end(); it++) {
        int v_id = it->first;
        Vehicle car = it->second;
        cars_order[car.s] = v_id;
    }

    // Separate cars by lane
    map<double, int>::iterator itr;
    for (itr = cars_order.begin(); itr != cars_order.end(); itr++) {
        double s = itr->first;
        int v_id = itr->second;
        Vehicle car = vehicles[v_id];

        switch (car.lane) {
            case 0: 
                cout << "[" << v_id << "]: s: " << car.s << " v: " << car.v << " l: " << car.lane << endl;
                break;
            case 1:
                cout << "\t\t\t\t\t[" << v_id << "]: s: " << car.s << " v: " << car.v << " l: " << car.lane << endl;
                break;
            case 2:
                cout << "\t\t\t\t\t\t\t\t\t\t[" << v_id << "]: s: " << car.s << " v: " << car.v << " l: " << car.lane << endl;
                break;
            default:
                cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t[" << v_id << "]: s: " << car.s << " v: " << car.v << " l: " << car.lane << endl;
         }
    }
}


void Vehicle::transition_function(map<int, Vehicle> &vehicles, Vehicle &ego) {

    // Get the data you need for the calculations to determine the next trajectory
    get_sensor_config(vehicles, ego);

    // Find the next state of possible trajectories
    string next_state = get_next_state(vehicles, ego);

    // Implement trajectory for next state
    implement_trajectory(vehicles, ego, next_state);

    


}


// Get the data you need for the calculations to determine the next trajectory
// Find cars ahead and behind of ego car
// Find nearest car ahead and nearest car behind ego car
// Find s, v of nearest cars
// void Vehicle::get_sensor_config(map<int, Vehicle> &vehicles, Vehicle &ego, double MAX_VEL, int NUM_LANES, double SPEED_LIMIT) {
void Vehicle::get_sensor_config(map<int, Vehicle> &vehicles, Vehicle &ego) {
   
    map <int, Vehicle> cars_ahead;
    map <int, Vehicle> cars_behind;
    map<int, Vehicle>::iterator it;
    for (it = vehicles.begin(); it != vehicles.end(); it++) {
        int v_id = it->first;
        Vehicle car = it->second;

        // SEPARATE CARS AHEAD AND BEHIND EGO CAR
        if (car.s > ego.s) {
            cars_ahead[v_id] = car;
        } else {
            cars_behind[v_id] = car;
        }
    }

    // NEAREST CARS TO EGO CAR
    this->lane_nearest_cars_ahead = lane_nearest_cars(vehicles, cars_ahead, ego);
    this->lane_nearest_cars_behind = lane_nearest_cars(vehicles, cars_behind, ego);

    // NEAREST S TO EGO CAR
    // cout << "Nearest S ahead L0: " << lane_nearest_cars_ahead[0] << "\t=============== Nearest S ahead L1: " << lane_nearest_cars_ahead[1] << "\t=================" << "Nearest S ahead L2: " << lane_nearest_cars_ahead[2] << "\t===================\n";
    // cout << "Nearest S behind L0: " << lane_nearest_cars_behind[0] << "\t=============== Nearest S behind L1: " << lane_nearest_cars_behind[1] << "\t=================" << "Nearest S behind L2: " << lane_nearest_cars_behind[2] << "\t===================\n";

    // V OF NEAREST S TO EGO CAR
    // cout << "Nearest V ahead L0: " << vehicles[lane_nearest_cars_ahead[0] ].v << "\t========== Nearest V ahead L1: " << vehicles[lane_nearest_cars_ahead[1] ].v << "\t================= Nearest V ahead L2: " << vehicles[lane_nearest_cars_ahead[2] ].v << "\t=========\n";
    // cout << "Nearest V behind L0: " << vehicles[lane_nearest_cars_behind[0] ].v << "\t========== Nearest V behind L1: " << vehicles[lane_nearest_cars_behind[1] ].v << "\t================= Nearest V behind L2: " << vehicles[lane_nearest_cars_behind[2] ].v << "\t=========\n";

}


// CALCULATES THE DISTANCE BETWEEN EACH CAR AND EGO CAR IN EACH LANE AND 
// RETURNS A VECTOR OF INDICES. THIS IS USED TO CALCULATE THE COST OF LANE CHANGE.
vector <int> Vehicle::lane_nearest_cars(map<int, Vehicle> &vehicles, map<int, Vehicle> &cars, Vehicle &ego) {

    // Sort cars ahead of ego by pos
    map <double, int> cars_order;
    map<int, Vehicle>::iterator it;
    for (it = cars.begin(); it != cars.end(); it++) {
        int v_id = it->first;
        Vehicle car = it->second;
        cars_order[car.s] = v_id;
    }

    double min_s_L0 = 500.0;
    double min_s_L1 = 500.0;
    double min_s_L2 = 500.0;
    int min_id_s_L0 = -1;
    int min_id_s_L1 = -1;
    int min_id_s_L2 = -1;

    double distance;

    map<double, int>::iterator itr;
    for (itr = cars_order.begin(); itr != cars_order.end(); itr++) {
        double s = itr->first;
        int v_id = itr->second;
        Vehicle car = vehicles[v_id];
        distance = fabs(car.s - ego.s);

        switch (car.lane) {
            case 0: 
                if (distance < min_s_L0) {
                    min_s_L0 = distance; 
                    min_id_s_L0 = v_id;  
                }              
                break;
            case 1:
                if (distance < min_s_L1) {
                    min_s_L1 = distance; 
                    min_id_s_L1 = v_id;  
                }              
                break;
            case 2:
                if (distance < min_s_L2) {
                    min_s_L2 = distance; 
                    min_id_s_L2 = v_id;  
                }              
                break;
        }
    }

    return {min_id_s_L0, min_id_s_L1, min_id_s_L2};
}


string Vehicle::get_next_state(map<int, Vehicle> &vehicles, Vehicle &ego) {

    string state;

    // This will create the map of possible states to follow a current state.
    // The ego car default state id "KL".
    get_possible_states();
    vector<string> next_possible_states = possible_states[ego.state];

    // Calculate the cost of each possible next state
    vector <double> costs = calculate_COST (vehicles, ego);

    // Compare and get the minimum cost
    int next_lane = min_element(costs.begin(), costs.end()) - costs.begin();

    // If the next_lane's possible state to transition to is part of the list 
    // of ego state's next_possible_states, then it can be the next_state.
    // If cost between lanes for changing are equal, default tot "KL".
    cout << "Current lane:\t" << ego.lane << endl;
    if (next_lane - ego.lane == 1) {
        if (ego.lane != NUM_LANES - 1) {
            if (find(next_possible_states.begin(), next_possible_states.end(), "PCLR") != next_possible_states.end()) {
                state = "PCLR";                            
            } 
        }
    } else if (next_lane - ego.lane == -1) {
        if (ego.lane != 0) {
            if (find(next_possible_states.begin(), next_possible_states.end(), "PCLL") != next_possible_states.end()) {
                state = "PCLL";
            } 
        }
    } else {
        state = "KL"; 
    } 

    // return next_state;
    return state;

}

// Create map of possible states the ego car can transition after its current state
void Vehicle::get_possible_states(){
    map<string, vector<string> > possible_states_after;
    possible_states_after["KL"] = {"KL", "PCLL", "PCLR"};
    possible_states_after["PCLL"] = {"KL", "PCLL", "CLL"};
    possible_states_after["PCLR"] = {"KL", "PCLR", "CLR"};
    possible_states_after["CLL"] = {"KL"};
    possible_states_after["CLR"] = {"KL"};
    this->possible_states = possible_states_after;
}


void Vehicle::implement_trajectory(map<int, Vehicle> &vehicles, Vehicle &ego, string next_state) {

    vector <string> poss_states = {"KL", "PCLL", "CLL", "PCLR", "CLR"};
    int index = find(poss_states.begin(), poss_states.end(), next_state) - poss_states.begin();
    if (index != poss_states.size()) {
        switch (index) {
           case 1 :
//                 if (ego.v < MAX_VEL) 
//                     ego.v += ego.a;
//                 else
//                     ego.v -= ego.a;
//                 ego.lane -= 1;
                cout << "Now state: " << poss_states[1] << endl;
                implement_trajectory_PCLL(vehicles, ego);
                break;
            case 2 :
//                 ego.lane -= 1;
//                 if (ego.v < MAX_VEL) 
//                     ego.v += ego.a;
//                 else
//                     ego.v -= ego.a;
                cout << "Now state: " << poss_states[2] << endl;
                implement_trajectory_CLL(vehicles, ego);
                break;
            case 3 :
//                 if (ego.v < MAX_VEL) 
//                     ego.v += ego.a;
//                 else
//                     ego.v -= ego.a;
//                 ego.lane += 1;
                cout << "Now state: " << poss_states[3] << endl;
                implement_trajectory_PCLR(vehicles, ego);
                break;
            case 4 :
//                 if (ego.v < MAX_VEL) 
//                     ego.v += ego.a;
//                 else
//                     ego.v -= ego.a;
//                 cout << "Now state: " << poss_states[4] << endl;
//                 ego.lane += 1;
                cout << "Now state: " << poss_states[4] << endl;
                implement_trajectory_CLR(vehicles, ego);
                break;
            default:
//                 if (ego.v < MAX_VEL) 
//                     ego.v += ego.a;
//                 else
//                     ego.v -= ego.a;
                cout << "Now state: " << poss_states[0] << endl;
                implement_trajectory_KL(vehicles, ego);
         }
    }
    

}

void Vehicle::implement_trajectory_KL(map<int, Vehicle> &vehicles, Vehicle &ego) {

    double v_id = ego.lane_nearest_cars_ahead[ego.lane];
    // If there is a car in front
    if (v_id != -1) {
        // Calculate where ego car should be behind car ahead
        double s_car_ahead = vehicles[v_id].s;
        double next_s = s_car_ahead - DIST_BUFFER;
        cout << next_s << endl;
        
        // Calculate ego car speed behind car ahead
        double v_car_ahead = vehicles[v_id].v;
        
        // Calculate distance to be maintained by ego car while driving behind car ahead
        double delta_s = next s - ego.s;
        double delta_v = v_car_ahead - ego.v;
        double delta_t = delta_s / delta_v;
        ego.a = delta_v / delta_t;        
    } 
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
}

void Vehicle::implement_trajectory_PCLL(map<int, Vehicle> &vehicles, Vehicle &ego) {
    ego.lane -= 1;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
}
void Vehicle::implement_trajectory_CLL(map<int, Vehicle> &vehicles, Vehicle &ego) {
    ego.lane -= 1;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
}
void Vehicle::implement_trajectory_PCLR(map<int, Vehicle> &vehicles, Vehicle &ego) {
    ego.lane += 1;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
}
void Vehicle::implement_trajectory_CLR(map<int, Vehicle> &vehicles, Vehicle &ego) {
    ego.lane += 1;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
}


// =========================================================================
// =========================================================================


    // COST OF NOT STAYING IN THE MIDDLE OF THE LANE
    // double stay_in_center = stay_in_lane_center(ego);

    // cout << "=========================================" << "Ego Car D from center: " << fabs(ego.d - (ego.lane * 4 + 2)) << "\t=============================================================\n";
    // cout << "=========================================" << "Ego cost from center: " << stay_in_center << "\t=============================================================\n";


    // Add lane speed costs and lane distance costs
    // vector <double> calculate_cost_ahead;
    // vector <double> calculate_cost_behind;
    // transform (lanes_speed_cost_ahead.begin(), lanes_speed_cost_ahead.end(), lanes_distance_cost_ahead.begin(), back_inserter(calculate_cost_ahead), plus<double>());
    // transform (lanes_speed_cost_behind.begin(), lanes_speed_cost_behind.end(), lanes_distance_cost_behind.begin(), back_inserter(calculate_cost_behind), plus<double>());
// 
    // cout << "Cost L0: " << calculate_cost_ahead[0] << "\t========================" << "Cost L1: " << calculate_cost_ahead[1] << "\t===============" << "Cost L2: " << calculate_cost_ahead[2] << "\t===================\n";
    // cout << "Cost L0: " << calculate_cost_behind[0] << "\t========================" << "Cost L1: " << calculate_cost_behind[1] << "\t===============" << "Cost L2: " << calculate_cost_behind[2] << "\t===================\n";

    // vector <double> calculate_cost;
    // transform (calculate_cost_ahead.begin(), calculate_cost_ahead.end(), calculate_cost_behind.begin(), back_inserter(calculate_cost), plus<double>());

    // ego.lane = min_element(calculate_cost.begin(), calculate_cost.end()) - calculate_cost.begin();
    // ego.v = max_element(ave_vel_ahead.begin(), ave_vel_ahead.end()) - ave_vel_ahead.begin();


    // cout << "V of nearest S ahead L0: " << lane_nearest_s_ahead[0] << "\t====" << "V of nearest S ahead L1: " << lane_nearest_s_ahead[1] << "\t====" << "V of nearest S ahead L2: " << lane_nearest_s_ahead[2] << "\t===================\n";
    // cout << "Nearest S behind L0: " << lane_nearest_s_behind[0] << "\t====" << "Nearest S behind L1: " << lane_nearest_s_behind[0] << "\t====" << "Nearest S behind L2: " << lane_nearest_s_behind[0] << "\t===================\n";
    // cout << "[" << max_vel_id << "] at L" << vehicles[max_vel_id].lane << " V max: " << max_vel << "\t=============================================================\t[" << max_id_ahead << "] at L" << vehicles[max_id_ahead].lane << " S max ahead: " << max_dist_ahead << "\n";
    // cout << "[" << min_vel_id << "] at L" << vehicles[min_vel_id].lane << " V min: " << min_vel << "\t=============================================================\t[" << max_id_behind << "] at L" << vehicles[max_id_behind].lane << " S max behind: " << max_dist_behind << "\n";

    // GET AVERAGE LANE VELOCITY
    // lane_nearest_vel_ahead = lane_nearest_vel(vehicles, cars_ahead, MAX_VEL);
    // lane_nearest_vel_behind = lane_nearest_vel(vehicles, cars_behind, MAX_VEL);

    // cout << "Ave V ahead L0: " << ave_vel_ahead[0] << "\t==================" << "Ave V ahead L1: " << ave_vel_ahead[1] << "\t======================" << "Ave V ahead L2: " << ave_vel_ahead[2] << "\t===================\n";
    // cout << "Ave V behind L0: " << ave_vel_behind[0] << "\t==================" << "Ave V behind L1: " << ave_vel_behind[1] << "\t======================" << "Ave V behind L2: " << ave_vel_behind[2] << "\t===================\n";

    

    // cout << "Ave V L0: " << ave_v_0 << "\t==================" << "Ave V L1: " << ave_v_1 << "\t======================" << "Ave V L2: " << ave_v_2 << "\t===================\n";

    // vector <double> lanes_distance_cost_ahead = lanes_distance(lane_nearest_s_ahead, DIST_BUFFER);
    // vector <double> lanes_distance_cost_behind = lanes_distance(lane_nearest_s_behind, DIST_BUFFER);

    // vector <double> lanes_speed_cost_ahead = lanes_speed_ahead(ave_vel_ahead, MAX_VEL, SPEED_LIMIT);
    // vector <double> lanes_speed_cost_behind = lanes_speed_behind(ave_vel_behind, MAX_VEL, SPEED_LIMIT);


// // CALCULATES THE DISTANCE OF EGO CAR FROM THE CENTER OF THE LANE.
// // IT KEEPS THE CAR FROM SWERVING. 
// double Vehicle::d_within_lane(Vehicle &ego, double MAX_DS) {
//     // Sort cars ahead of ego by pos
//     map <double, int> cars_order;
//     map<int, Vehicle>::iterator it;
//     for (it = cars.begin(); it != cars.end(); it++) {
//         int v_id = it->first;
//         Vehicle car = it->second;
//         cars_order[car.s] = v_id;
//     }

//     vector <double> s_0;
//     vector <double> s_1;
//     vector <double> s_2;
//     double min_s_L0 = MAX_DS;
//     double min_s_L1 = MAX_DS;
//     double min_s_L2 = MAX_DS;
//     map<double, int>::iterator itr;
//     double distance;
//     for (itr = cars_order.begin(); itr != cars_order.end(); itr++) {
//         double s = itr->first;
//         int v_id = itr->second;
//         Vehicle car = vehicles[v_id];
//         distance = fabs(car.s - ego.s);

//         switch (car.lane) {
//             case 0: 
//                 s_0.push_back(distance);                 
//                 break;
//             case 1:
//                 s_1.push_back(distance);
//                 break;
//             case 2:
//                 s_2.push_back(distance);
//                 break;
//             default:
//                 cout << "\t\t\t\t\t\t\t\t\t\t\t\tnot in lane\n";
//         }
//     }

//     if (s_0.size() > 0) min_s_L0 = *min_element(s_0.begin(), s_0.end());
//     if (s_1.size() > 0) min_s_L1 = *min_element(s_1.begin(), s_1.end());
//     if (s_2.size() > 0) min_s_L2 = *min_element(s_2.begin(), s_2.end());

//     return {min_s_L0, min_s_L1, min_s_L2};
// }

// bool Vehicle::get_vehicle(map<int, Vehicle> &vehicles, Vehicle &ego, int lane, int prev_size, double DELTA_T, double MAX_DIST, Vehicle & rVehicle) {
//     /*
//     Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
//     rVehicle is updated if a vehicle is found.
//     */
//     bool found_vehicle = false;
//     int closest_car;
//     double closest_car_dist = MAX_DIST;

//     map<int, Vehicle>::iterator it = vehicles.begin();
//     while(it != vehicles.end()) {
//         int v_id = it->first;
//         Vehicle possible_car_detected = it->second;
//         if (possible_car_detected.lane == lane) {
//            // if using previous points can project s value out
//             possible_car_detected.s += ((double)prev_size * DELTA_T * mph_to_mps(possible_car_detected.v));
//             if (((possible_car_detected.s > ego.s) && ((possible_car_detected.s - ego.s) < MAX_DIST)) || (lane != ego.lane && (fabs(possible_car_detected.s - ego.s) < MAX_DIST))) {
//                 found_vehicle = true;
//                 if ((possible_car_detected.s - ego.s) < closest_car_dist) {
//                     closest_car = v_id;

//                     // CHECK
//                     // cout << "possible_car_detected: " << v_id << endl;
//                     // cout << "ego.s: " << ego.s << endl;
//                     closest_car_dist = possible_car_detected.s - ego.v;
//                 } 
//             }
//         }
//         it++;
//     }

//     if (found_vehicle) {
//         rVehicle = vehicles[closest_car];

//         // CHECK
//         // cout << "Car detected: " << closest_car << "\n"; // id of the car in front
//         // cout << "Car detected.s: " << rVehicle.s << endl;
//         // cout << "Car detected.v: " << rVehicle.v << endl;
//     }

//     return found_vehicle;
// }



