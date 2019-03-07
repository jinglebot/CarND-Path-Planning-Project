#include <algorithm>
#include <iostream>
#include <iomanip>
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

    see_cars_ahead(vehicles, cars_ahead);
    cout << endl;
    if (ego.lane == 0)
        cout << "= Ego Car S: " << ego.s << " V: " << ego.v << " L: " << ego.lane << " A: " << ego.a << "\t==========================================================================\n";
    if (ego.lane == 1)
        cout << "=================================== Ego Car S: " << ego.s << " V: " << ego.v << " L: " << ego.lane << " A: " << ego.a << "\t============================================\n";
    if (ego.lane == 2)
        cout << "================================================================================ Ego Car S: " << ego.s << " V: " << ego.v << " L: " << ego.lane << " A: " << ego.a << "\n";
    cout << endl;
    see_cars_behind(vehicles, cars_behind);
    cout << endl;
}


void Vehicle::see_cars_ahead(map<int, Vehicle> &vehicles, map<int, Vehicle> &cars) {

    // List cars ahead/behind of ego by pos
    map <double, int> cars_order;
    map<int, Vehicle>::iterator it;
    for (it = cars.begin(); it != cars.end(); it++) {
        int v_id = it->first;
        Vehicle car = it->second;
        cars_order[car.s] = v_id;
    }

    // Separate cars by lane
    map <double, int> lane_0;
    map <double, int> lane_1;
    map <double, int> lane_2;

    map<double, int>::iterator itr;
    for (itr = cars_order.begin(); itr != cars_order.end(); itr++) {
        double s = itr->first;
        int v_id = itr->second;
        Vehicle car = vehicles[v_id];

        switch (car.lane) {
            case 0: 
               lane_0[car.s] = v_id;
                break;
            case 1:
                 lane_1[car.s] = v_id;
                break;
            case 2:
                lane_2[car.s] = v_id;
                break;
        }
    }

   if (lane_0.size() == 0) {
        Vehicle vehicle = Vehicle(0, 500, MAX_VEL, 6);
        vehicles[-1] = vehicle;
        lane_0[500] = -1;        
    }
    if (lane_1.size() == 0) {
        Vehicle vehicle = Vehicle(1, 500, MAX_VEL, 6);
        vehicles[-1] = vehicle;        
        lane_1[500] = -1;        
    }
    if (lane_2.size() == 0) {
        Vehicle vehicle = Vehicle(2, 500, MAX_VEL, 6);
        vehicles[-1] = vehicle;        
        lane_2[500] = -1;        
    }

    for(auto it_0 = lane_0.rbegin(), end_0 = lane_0.rend(),
        it_1 = lane_1.rbegin(), end_1 = lane_1.rend(),
        it_2 = lane_2.rbegin(), end_2 = lane_2.rend();
        it_0 != end_0 || it_1 != end_1 || it_2 != end_2;)
    {
        string output0 = "================================";
        if(it_0 != end_0) {
            double s0 = it_0->first;
            int v_id0 = it_0->second;
            Vehicle car0 = vehicles[v_id0];
            output0 =  "[" + to_string(v_id0) + "]: s: " + to_string(s0) + " v: " + to_string(car0.v) + " l: " + to_string(car0.lane);
            ++it_0;
        }
        cout << setfill ('=') << setw(10) << output0 << "\t";

        string output1 = "================================";
        if(it_1 != end_1) {
            double s1 = it_1->first;
            int v_id1 = it_1->second;
            Vehicle car1 = vehicles[v_id1];
            output1 =  "[" + to_string(v_id1) + "]: s: " + to_string(s1) + " v: " + to_string(car1.v) + " l: " + to_string(car1.lane);
            ++it_1;
        }
        cout << setfill ('=') << setw(10) << output1 << "\t";

        string output2 = "================================";
        if(it_2 != end_2) {
            double s2 = it_2->first;
            int v_id2 = it_2->second;
            Vehicle car2 = vehicles[v_id2];
            output2 =  "[" + to_string(v_id2) + "]: s: " + to_string(s2) + " v: " + to_string(car2.v) + " l: " + to_string(car2.lane);
            ++it_2;
        }
        cout << setfill ('=') << setw(10) << output2 << "\t";
            
        cout << endl;
    }
}



void Vehicle::see_cars_behind(map<int, Vehicle> &vehicles, map<int, Vehicle> &cars) {

    // List cars ahead/behind of ego by pos
    map <double, int> cars_order;
    map<int, Vehicle>::iterator it;
    for (it = cars.begin(); it != cars.end(); it++) {
        int v_id = it->first;
        Vehicle car = it->second;
        cars_order[car.s] = v_id;
    }

    // Separate cars by lane
    map <double, int> lane_0;
    map <double, int> lane_1;
    map <double, int> lane_2;

    map<double, int>::iterator itr;
    for (itr = cars_order.begin(); itr != cars_order.end(); itr++) {
        double s = itr->first;
        int v_id = itr->second;
        Vehicle car = vehicles[v_id];

        switch (car.lane) {
            case 0: 
                lane_0[car.s] = v_id;
                break;
            case 1:
                lane_1[car.s] = v_id;
                break;
            case 2:
                lane_2[car.s] = v_id;
                break;
         }
    }

   if (lane_0.size() == 0) {
        Vehicle vehicle = Vehicle(0, 500, MAX_VEL, 6);
        vehicles[-1] = vehicle;
        lane_0[500] = -1;        
    }
    if (lane_1.size() == 0) {
        Vehicle vehicle = Vehicle(1, 500, MAX_VEL, 6);
        vehicles[-1] = vehicle;        
        lane_1[500] = -1;        
    }
    if (lane_2.size() == 0) {
        Vehicle vehicle = Vehicle(2, 500, MAX_VEL, 6);
        vehicles[-1] = vehicle;        
        lane_2[500] = -1;        
    }


    for(auto it_0 = lane_0.begin(), end_0 = lane_0.end(),
        it_1 = lane_1.begin(), end_1 = lane_1.end(),
        it_2 = lane_2.begin(), end_2 = lane_2.end();
        it_0 != end_0 || it_1 != end_1 || it_2 != end_2;)
    {
        string output0 = "================================";
        if(it_0 != end_0) {
            double s0 = it_0->first;
            int v_id0 = it_0->second;
            Vehicle car0 = vehicles[v_id0];
            output0 =  "[" + to_string(v_id0) + "]: s: " + to_string(s0) + " v: " + to_string(car0.v) + " l: " + to_string(car0.lane);
            ++it_0;
        }
        cout << setfill ('=') << setw(10) << output0 << "\t";

        string output1 = "================================";
        if(it_1 != end_1) {
            double s1 = it_1->first;
            int v_id1 = it_1->second;
            Vehicle car1 = vehicles[v_id1];
            output1 =  "[" + to_string(v_id1) + "]: s: " + to_string(s1) + " v: " + to_string(car1.v) + " l: " + to_string(car1.lane);
            ++it_1;
        }
        cout << setfill ('=') << setw(10) << output1 << "\t";

        string output2 = "================================";
        if(it_2 != end_2) {
            double s2 = it_2->first;
            int v_id2 = it_2->second;
            Vehicle car2 = vehicles[v_id2];
            output2 =  "[" + to_string(v_id2) + "]: s: " + to_string(s2) + " v: " + to_string(car2.v) + " l: " + to_string(car2.lane);
            ++it_2;
        }
        cout << setfill ('=') << setw(10) << output2 << "\t";
            
        cout << endl;
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
    // cout << "Nearest S ahead L0: " << vehicles[lane_nearest_cars_ahead[0] ].s - DIST_BUFFER << "\t=============== Nearest S ahead L1: " << vehicles[lane_nearest_cars_behind[1] ].s - DIST_BUFFER << "\t=================" << "Nearest S ahead L2: " << vehicles[lane_nearest_cars_behind[2] ].s - DIST_BUFFER << "\t===================\n";
    cout << "Nearest S ahead L0: " << lane_nearest_cars_ahead[0] << "\t========== Nearest S ahead L1: " << lane_nearest_cars_ahead[1] << "\t============" << "Nearest S ahead L2: " << lane_nearest_cars_ahead[2] << "\t==============\n";
    // cout << "Nearest S behind L0: " <<  ego.s - vehicles[lane_nearest_cars_behind[0] ].s << "\t=============== Nearest S behind L1: " << ego.s - vehicles[lane_nearest_cars_behind[1] ].s << "\t=================" << "Nearest S behind L2: " << ego.s - vehicles[lane_nearest_cars_behind[2] ].s << "\t===================\n";

    // V OF NEAREST S TO EGO CAR
    // cout << "Nearest V ahead L0: " << vehicles[lane_nearest_cars_ahead[0] ].v << "\t========== Nearest V ahead L1: " << vehicles[lane_nearest_cars_ahead[1] ].v << "\t================= Nearest V ahead L2: " << vehicles[lane_nearest_cars_ahead[2] ].v << "\t=========\n";
    // cout << "Nearest V behind L0: " << vehicles[lane_nearest_cars_behind[0] ].v << "\t========== Nearest V behind L1: " << vehicles[lane_nearest_cars_behind[1] ].v << "\t================= Nearest V behind L2: " << vehicles[lane_nearest_cars_behind[2] ].v << "\t=========\n";
    // cout << "Future S behind L0: " << get_future_car_position(vehicles[lane_nearest_cars_behind[0] ].v, vehicles[lane_nearest_cars_behind[0] ].s) << "\t========== Future S behind L1: " << get_future_car_position(vehicles[lane_nearest_cars_behind[1] ].v, vehicles[lane_nearest_cars_behind[1] ].s) << "\t================= Future S behind L2: " << get_future_car_position(vehicles[lane_nearest_cars_behind[2] ].v, vehicles[lane_nearest_cars_behind[2] ].s) << "\t=========\n";

}

double Vehicle::get_future_car_position(double car_v, double car_s) {
    double delta_t = MIN_TIME_LANE_CHANGE;
    double delta_s = car_v * delta_t;
    double future_s = car_s + delta_s;
    return future_s;
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

    if (min_s_L0 > MAX_DIST) 
        min_id_s_L0 = -1;
    if (min_s_L1 > MAX_DIST) 
        min_id_s_L1 = -1;
    if (min_s_L2 > MAX_DIST) 
        min_id_s_L2 = -1;

    return {min_id_s_L0, min_id_s_L1, min_id_s_L2};
}


string Vehicle::get_next_state(map<int, Vehicle> &vehicles, Vehicle &ego) {

    string state = ego.state;

    // This will create the map of possible states to follow a current state.
    // The ego car default state id "KL".
    get_possible_states();
    vector<string> next_possible_states = possible_states[ego.state];
    
    // Calculate the cost of each possible next state
    vector <double> costs = calculate_COST (vehicles, ego);

    // Compare and get the minimum cost
    int next_lane = min_element(costs.begin(), costs.end()) - costs.begin();

    cout << "Possible states: " ;
    for (string possible_state: next_possible_states) {
        cout << possible_state << "\t";
        double new_lane;
        if (possible_state == "KL") {
            new_lane = ego.lane;
            if (new_lane == next_lane)
                state = "KL";;        
        } 
        else if (possible_state == "PCLR") {
            new_lane = ego.lane + 1;
            if (new_lane == next_lane)
                state = "PCLR";;        
        }
        else if (possible_state == "PCLL") {
            new_lane = ego.lane - 1;
            if (new_lane == next_lane)
                state = "PCLL";;        
        }
        else if (possible_state == "CLR") {
            new_lane = ego.lane + 1;
            if (new_lane == next_lane)
                state = "CLR";;        
        }
        else if (possible_state == "CLL") {
            new_lane = ego.lane - 1;
            if (new_lane == next_lane)
                state = "CLL";;        
        }
    }
    cout << endl;





    // If the next_lane's possible state to transition to is part of the list 
    // of ego state's next_possible_states, then it can be the next_state.
    // If cost between lanes for changing are equal, default to "KL".
    // if (next_lane - ego.lane == 1) {
    //     if (ego.lane != NUM_LANES - 1) {
    //         if (find(next_possible_states.begin(), next_possible_states.end(), "PCLR") != next_possible_states.end() ) {
    //             if ( (vehicles[lane_nearest_cars_ahead[ego.lane] ].s - ego.s > DIST_BUFFER)  && find(next_possible_states.begin(), next_possible_states.end(), "PCLR") != next_possible_states.end() )
    //                 state = "CLR";
    //             else state = "PCLR";
    //         }
    //     }
    // } else if (next_lane - ego.lane == -1) {
    //     if (ego.lane != 0) {
    //         if (find(next_possible_states.begin(), next_possible_states.end(), "PCLL") != next_possible_states.end() ) {
    //             if ( (vehicles[lane_nearest_cars_ahead[ego.lane] ].s - ego.s > DIST_BUFFER) && (vehicles[lane_nearest_cars_ahead[ego.lane] ].v - ego.v > 0.675) && find(next_possible_states.begin(), next_possible_states.end(), "PCLL") != next_possible_states.end() )
    //                 state = "CLL";
    //             else state = "PCLL";
    //         } 
    //     }
    // } else state = "KL";
    return state;

}


// Create map of possible states the ego car can transition after its current state
void Vehicle::get_possible_states(){
    map<string, vector<string> > possible_states_after;
    possible_states_after["KL"] = {"KL", "PCLL", "PCLR"};
    possible_states_after["PCLL"] = {"PCLL", "CLL"};
    possible_states_after["PCLR"] = {"PCLR", "CLR"};
    possible_states_after["CLL"] = {"KL"};
    possible_states_after["CLR"] = {"KL"};
    this->possible_states = possible_states_after;
}


void Vehicle::implement_trajectory(map<int, Vehicle> &vehicles, Vehicle &ego, string next_state) {
    cout << "next_state: " << next_state << endl;
    vector <string> poss_states = {"KL", "PCLL", "CLL", "PCLR", "CLR"};
    int index = find(poss_states.begin(), poss_states.end(), next_state) - poss_states.begin();
    if (index != poss_states.size()) {
        switch (index) {
           case 1 :
                implement_trajectory_PCLL(vehicles, ego);
                break;
            case 2 :
                implement_trajectory_CLL(vehicles, ego);
                break;
            case 3 :
                implement_trajectory_PCLR(vehicles, ego);
                break;
            case 4 :
                implement_trajectory_CLR(vehicles, ego);
                break;
            default:
                implement_trajectory_KL(vehicles, ego);
         }
    }
    

}


void Vehicle::implement_trajectory_KL(map<int, Vehicle> &vehicles, Vehicle &ego) {

    // Calculate where ego car should be behind car ahead
    double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
    double s_car_ahead = vehicles[v_id_ahead].s;
    double next_s = s_car_ahead - DIST_BUFFER;
    
    // If there is a car in front
    if (v_id_ahead != -1) {
        
        // Calculate what ego car speed should be behind car ahead
        double v_car_ahead = vehicles[v_id_ahead].v;
    
        // Calculate distance and velocity to be maintained by ego car while driving behind car ahead
        // calculate for the final s of car ahead from delta_t
        // delta_t of ego from now to when ideally behind the car ahead == delta_t of car ahead from now to when ideally ego car is behind it
        // delta_t of car ahead == (s final - s now) / v, assuming acceleration == 0
        // delta_t of ego car == (s final of car ahead - DIST_BUFFER - ego.s) / (v_car_ahead - ego.v);
        // acceleration == (delta_v / delta_t) / N, N == delta_t / 0.02 increments
        // double s_car_ahead_f;
        double delta_v = mph_to_mps(v_car_ahead - ego.v);
        // s_car_ahead_f = (DIST_BUFFER + ego.s - (s_car_ahead * delta_v  / v_car_ahead) ) / (1 - delta_v  / v_car_ahead);
        // double next_s = s_car_ahead_f - DIST_BUFFER;
        double delta_s = next_s - ego.s;
        cout << "delta_s: " << delta_s << endl;

        double delta_t = fabs(delta_s / delta_v);
        cout << "delta_t: " << delta_t << endl;

        int N = fabs(delta_t / D_TIME);
        double a = fabs(delta_v) / delta_t;
       

        // Ego car does violate the speed limit sometimes
        // despite the limits placed
        // but only when it's catching up with the traffic ahead
        if (fabs(a) > MAX_ACCEL) ego.a = MAX_ACCEL;
        else ego.a = a;

        if ( (delta_s > DIST_BUFFER) && (ego.s < next_s ) && (ego.v < MAX_VEL) )
            ego.v += ego.a;
        else if (ego.v > v_car_ahead)
            ego.v -= ego.a;
        else 
            ego.v = v_car_ahead;
       
    // If there is no car in front
    } else {
        ego.a = MAX_ACCEL;
        if (ego.v < MAX_VEL) ego.v += ego.a;
        else ego.v -= ego.a;
    }
}


void Vehicle::implement_trajectory_PCLL(map<int, Vehicle> &vehicles, Vehicle &ego) {
    int next_lane = ego.lane - 1;
    double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
    double s_car_ahead = vehicles[v_id_ahead].s;
    double next_s_ahead = s_car_ahead - DIST_BUFFER;
    
    // Calculate distance ego car should be behind car in next lane
    double v_id_next_lane = this->lane_nearest_cars_ahead[next_lane];
    double s_car_next_lane = vehicles[v_id_next_lane].s;
    double next_s_next_lane = s_car_next_lane - DIST_BUFFER;
    // cout << "next_s: " << next_s << endl;

    double next_s = min(next_s_ahead, next_s_next_lane);
    if ( (next_s - ego.s) < 1) {
        ego.state = "CLL";
    }

    // If there is a car in front
    if (v_id_ahead != -1) {
    
        // Calculate what ego car speed should be behind car ahead
        double v_car_ahead = vehicles[v_id_ahead].v;

        // Calculate distance and velocity to be maintained by ego car while driving behind car ahead
       double delta_v = mph_to_mps(v_car_ahead - ego.v);
        double delta_s = next_s - ego.s;
        cout << "delta_s: " << delta_s << endl;

        double delta_t = fabs(delta_s / delta_v);
        cout << "delta_t: " << delta_t << endl;

        // int N = fabs(delta_t / D_TIME);
        double a = fabs(delta_v) / delta_t;
       
        if (fabs(a) > MAX_ACCEL) 
            ego.a = MAX_ACCEL;
        else 
            ego.a = a;

        if ( (delta_s > DIST_BUFFER) && (ego.s < next_s ) && (ego.v < MAX_VEL) )
            ego.v += ego.a;
        else if (ego.v > v_car_ahead)
            ego.v -= ego.a;
        else 
            ego.v = v_car_ahead;

    // If there is no car in front
    } else {
        ego.a = MAX_ACCEL;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
    }
}


void Vehicle::implement_trajectory_CLL(map<int, Vehicle> &vehicles, Vehicle &ego) {
    int next_lane = ego.lane - 1;

    // Calculate distance ego car should be behind car in next lane
    double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
    double s_car_ahead = vehicles[v_id_ahead].s;
    double next_s_ahead = s_car_ahead - DIST_BUFFER;
    // cout << "next_s: " << next_s << endl;

    // Calculate distance ego car should be behind car in next lane
    double v_id_next_lane = this->lane_nearest_cars_ahead[next_lane];
    double s_car_next_lane = vehicles[v_id_next_lane].s;
    double next_s_next_lane = s_car_next_lane - DIST_BUFFER;
    // cout << "next_s: " << next_s << endl;

    // Calculate which lane ego car should be behind car ahead
    // cout << "initial d: " << ego.d << endl;
    // cout << "initial lane: " << ego.lane << endl;
    double next_d = next_lane * 4 + 2;
    ego.d = next_d;
    ego.lane = next_lane;
    // cout << "target d: " << next_d << endl;
    // cout << "target lane: " << next_lane << endl;

//     // double v_final = MAX_ACCEL * MIN_TIME_LANE_CHANGE + ego.v;
//     // cout << "target vel limit: " << v_final << endl;

    // If there is a car in front
    if (v_id_next_lane != -1) {

        double v_car_next_lane = vehicles[v_id_next_lane].v;
//         // cout << "ego.v: " << ego.v << endl;
//         // cout << "v_car_next_lane: " << v_car_next_lane << endl;

        double delta_s = next_s_next_lane - ego.s;
//         // cout << "delta_d: " << delta_d << endl;
        double delta_v = mph_to_mps(v_car_next_lane - ego.v);
//         // cout << "delta_v: " << delta_v << endl;
        double delta_t = fabs(delta_s / delta_v);
//         // cout << "delta_t: " << delta_t << endl;

//         if (delta_t > MIN_TIME_LANE_CHANGE) 
//             delta_t = MIN_TIME_LANE_CHANGE;
        

//         int N = fabs(delta_t / D_TIME);
//         // cout << "N: " << N << endl;
            
        double a = fabs(delta_v) / delta_t;
           
        if (fabs(a) > MAX_ACCEL) 
            ego.a = MAX_ACCEL;
        else 
            ego.a = a;

        // cout << "ego.a: " << ego.a << endl;

        if ( (delta_s > DIST_BUFFER) && (ego.s < next_s_next_lane ) && (ego.v < MAX_VEL) )
            ego.v += ego.a;
        else if (ego.v > v_car_next_lane)
            ego.v -= ego.a;
        else 
            ego.v = v_car_next_lane;

    // If there is no car in front
    } else {
        ego.a = MAX_ACCEL;
        // cout << "ego.a: " << ego.a << endl;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
    }
}


void Vehicle::implement_trajectory_PCLR(map<int, Vehicle> &vehicles, Vehicle &ego) {
    int next_lane = ego.lane + 1;

    // Calculate which lane ego car should be behind car ahead
    double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
    double s_car_ahead = vehicles[v_id_ahead].s;
    double next_s_ahead = s_car_ahead - DIST_BUFFER;
    // cout << "next_s: " << next_s << endl;

    // Calculate distance ego car should be behind car in next lane
    double v_id_next_lane = this->lane_nearest_cars_ahead[next_lane];
    double s_car_next_lane = vehicles[v_id_next_lane].s;
    double next_s_next_lane = s_car_next_lane - DIST_BUFFER;
    // cout << "next_s: " << next_s << endl;

    // double next_s = min(next_s_ahead, next_s_next_lane);
    // if ( (next_s - ego.s) < 1) {
    //     ego.state = "CLR";
    // }

    
    // If there is a car in front
    if (v_id_ahead != -1) {
    
        double v_car_next_lane = vehicles[v_id_next_lane].v;
        double delta_v = mph_to_mps(v_car_next_lane - ego.v);
        double delta_t = MIN_TIME_LANE_CHANGE;
        double a = fabs(delta_v) / delta_t;

        if (fabs(a) > MAX_ACCEL) 
            a = MAX_ACCEL;

        // Find the minimum delta_s to change lane within 3 secs
        // S = Vo * t + 0.5 * a * t * t
        double delta_s = mph_to_mps(ego.v) * delta_t + 0.5 * a * delta_t * delta_t;
        double s_limit = next_s_ahead - delta_s;
        double s_space = ego.s + delta_s;
        int N = delta_t / D_TIME;

        if ( (ego.s < s_limit) || (next_s_ahead > s_space) ) {
            ego.state = "CLR";
        }
       
        if ( ( (s_car_ahead - ego.s) > DIST_BUFFER) && (ego.v < MAX_VEL) )
            ego.v += ego.a;
        else if (ego.v > v_car_ahead)
            ego.v -= ego.a;
        else 
            ego.v = v_car_ahead;

    // If there is no car in front
    } else {
        ego.a = MAX_ACCEL;
        if (ego.v < MAX_VEL) 
            ego.v += ego.a;
        else
            ego.v -= ego.a;
    }
}



void Vehicle::implement_trajectory_CLR(map<int, Vehicle> &vehicles, Vehicle &ego) {
    int next_lane = ego.lane + 1;
    
    // Start
    double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
    double s_car_ahead = vehicles[v_id_ahead].s;
    double next_s_ahead = s_car_ahead - DIST_BUFFER;
    // cout << "next_s: " << next_s << endl;

    // Goal
    double v_id_next_lane = this->lane_nearest_cars_ahead[next_lane];
    double s_car_next_lane = vehicles[v_id_next_lane].s;

    double goal_s = s_car_next_lane - DIST_BUFFER;



    // If there is a car in front in the next lane
    if (v_id_next_lane != -1) {

        double v_car_next_lane = vehicles[v_id_next_lane].v;
        double delta_v = v_car_next_lane - ego.v;
        double delta_t = MIN_TIME_LANE_CHANGE;
        double a = fabs(delta_v) / delta_t;

        // S = Vo * t + 0.5 * a * t * t
        double delta_s = ego.v * delta_t + 0.5 * a * delta_t * delta_t;
        double starting_s = goal_s - delta_s;
        int N = delta_t / D_TIME;

 



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
            double target_x = delta_s; // in meters of spaced points
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


//         double delta_v = delta_d / delta_t;        
           
//         if (fabs(a) > MAX_ACCEL) ego.a = MAX_ACCEL;
//         else ego.a = a;

//         if ( (ego.v < v_car_next_lane) && (ego.s < next_s_next_lane ) && (ego.v < MAX_VEL) )
//             ego.v += ego.a;
//         else ego.v -= ego.a;

//     // If there is no car in front
//     } else {
//         ego.a = MAX_ACCEL;
//         if (ego.v < MAX_VEL) ego.v += ego.a;
//         else ego.v -= ego.a;
//     }
// }



// void Vehicle::implement_trajectory_CLL(map<int, Vehicle> &vehicles, Vehicle &ego) {
//     int next_lane = ego.lane - 1;
//     double v_id_next_lane = ego.lane_nearest_cars_ahead[next_lane];
//     double s_car_next_lane = vehicles[v_id_next_lane].s;
//     double next_s_next_lane = s_car_next_lane - DIST_BUFFER;
//     double next_d = next_lane * 4 + 2;
//     ego.d = next_d;
//     ego.lane = next_lane;

//     // If there is a car in front
//     if (v_id_next_lane != -1) {

//         double v_car_next_lane = vehicles[v_id_next_lane].v;
//         double delta_d = next_d - ego.d;
//         double delta_t = MIN_TIME_LANE_CHANGE;
//         double delta_v = delta_d / delta_t;        
//         int N = fabs(delta_t / D_TIME);
//         double a = fabs(delta_v) / (delta_t * N);
           
//         if (fabs(a) > MAX_ACCEL) ego.a = MAX_ACCEL;
//         else ego.a = a;

//         if ( (ego.v < v_car_next_lane) && (ego.s < next_s_next_lane ) && (ego.v < MAX_VEL) )
//             ego.v += ego.a;
//         else ego.v -= ego.a;

//     // If there is no car in front
//     } else {
//         ego.a = MAX_ACCEL;
//         if (ego.v < MAX_VEL) ego.v += ego.a;
//         else ego.v -= ego.a;
//     }
// }


// void Vehicle::implement_trajectory_PCLR(map<int, Vehicle> &vehicles, Vehicle &ego) {
//     int next_lane = ego.lane + 1;

//     // Calculate which lane ego car should be behind car ahead
//     double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
//     double s_car_ahead = vehicles[v_id_ahead].s;
//     double next_s_ahead = s_car_ahead - DIST_BUFFER;
//     // cout << "next_s: " << next_s << endl;

//     // Calculate distance ego car should be behind car in next lane
//     double v_id_next_lane = this->lane_nearest_cars_ahead[next_lane];
//     double s_car_next_lane = vehicles[v_id_next_lane].s;
//     double next_s_next_lane = s_car_next_lane - DIST_BUFFER;
//     // cout << "next_s: " << next_s << endl;

//     double next_s = min(next_s_ahead, next_s_next_lane);
//     if ( (next_s - ego.s) < 1) {
//         ego.state = "CLR";
//     }

    
//     // If there is a car in front
//     if (v_id_ahead != -1) {
    
//         // Calculate what ego car speed should be behind car ahead
//         double v_car_ahead = vehicles[v_id_ahead].v;

//         // Calculate distance and velocity to be maintained by ego car while driving behind car ahead
//         double delta_v = mph_to_mps(v_car_ahead - ego.v);
//         double delta_s = next_s - ego.s;
//         cout << "delta_s: " << delta_s << endl;
    
//         double delta_t = fabs(delta_s / delta_v);
//         cout << "delta_t: " << delta_t << endl;

//         // int N = fabs(delta_t / D_TIME);
//         double a = fabs(delta_v) / delta_t;
       
//         if (fabs(a) > MAX_ACCEL) 
//             ego.a = MAX_ACCEL;
//         else 
//             ego.a = a;

//         if ( ( (s_car_ahead - ego.s) > DIST_BUFFER) && (delta_s > LANE_CHANGE_BUFFER_AHEAD ) && (ego.v < MAX_VEL) )
//             ego.v += ego.a;
//         else if (ego.v > v_car_ahead)
//             ego.v -= ego.a;
//         else 
//             ego.v = v_car_ahead;

//     // If there is no car in front
//     } else {
//         ego.a = MAX_ACCEL;
//         if (ego.v < MAX_VEL) 
//             ego.v += ego.a;
//         else
//             ego.v -= ego.a;
//     }
// }


// void Vehicle::implement_trajectory_CLR(map<int, Vehicle> &vehicles, Vehicle &ego) {
//     int next_lane = ego.lane + 1;

//     // Calculate distance ego car should be behind car in next lane
//     double v_id_ahead = this->lane_nearest_cars_ahead[ego.lane];
//     double s_car_ahead = vehicles[v_id_ahead].s;
//     double next_s_ahead = s_car_ahead - DIST_BUFFER;
//     // cout << "next_s: " << next_s << endl;

//     // Calculate distance ego car should be behind car in next lane
//     double v_id_next_lane = this->lane_nearest_cars_ahead[next_lane];
//     double s_car_next_lane = vehicles[v_id_next_lane].s;
//     double next_s_next_lane = s_car_next_lane - DIST_BUFFER;
//     // cout << "next_s: " << next_s << endl;

//     // Calculate which lane ego car should be behind car ahead
// //     // cout << "initial d: " << ego.d << endl;
// //     // cout << "initial lane: " << ego.lane << endl;
//     double next_d = next_lane * 4 + 2;
//     ego.d = next_d;
//     ego.lane = next_lane;
// //     // cout << "target d: " << next_d << endl;
// //     // cout << "target lane: " << next_lane << endl;


// //     // double v_final = MAX_ACCEL * MIN_TIME_LANE_CHANGE + ego.v;
// //     // cout << "target vel limit: " << v_final << endl;

//     // If there is a car in front
//     if (v_id_next_lane != -1) {

//         double v_car_next_lane = vehicles[v_id_next_lane].v;
// //         // cout << "ego.v: " << ego.v << endl;
// //         // cout << "v_car_next_lane: " << v_car_next_lane << endl;

//         double delta_s = next_s_next_lane - ego.s;
// //         // cout << "delta_d: " << delta_d << endl;
//         double delta_v = mph_to_mps(v_car_next_lane - ego.v);
// //         // cout << "delta_v: " << delta_v << endl;
//         double delta_t = fabs(delta_s / delta_v);
// //         // cout << "delta_t: " << delta_t << endl;

// //         if (delta_t > MIN_TIME_LANE_CHANGE) 
// //             delta_t = MIN_TIME_LANE_CHANGE;
        

// //         int N = fabs(delta_t / D_TIME);
// //         // cout << "N: " << N << endl;
            
//         double a = fabs(delta_v) / delta_t;
           
//         if (fabs(a) > MAX_ACCEL) 
//             ego.a = MAX_ACCEL;
//         else 
//             ego.a = a;

//         // cout << "ego.a: " << ego.a << endl;

//         if ( ( (s_car_ahead - ego.s) > DIST_BUFFER) && (delta_s > LANE_CHANGE_BUFFER_AHEAD ) && (ego.v < MAX_VEL) )
//             ego.v += ego.a;
//         else if (ego.v > v_car_next_lane)
//             ego.v -= ego.a;
//         else 
//             ego.v = v_car_next_lane;

//     // If there is no car in front
//     } else {
//         ego.a = MAX_ACCEL;
//         // cout << "ego.a: " << ego.a << endl;
//         if (ego.v < MAX_VEL) 
//             ego.v += ego.a;
//         else
//             ego.v -= ego.a;
//     }
// }

