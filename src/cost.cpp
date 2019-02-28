#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "cost.h"
#include "helper.h"

using namespace std;


// COST OF SPEEDING UP AND SLOWING DOWN OF CARS AHEAD
vector <double> lanes_speed_ahead(vector <double> &ave_vel, double ego_vel) {
    vector<double> costs;
    for (int i = 0; i < ave_vel.size(); i++) {
        double cost = lane_speed_ahead(ave_vel[i], ego_vel);
        costs.push_back(cost);
    }
    return costs;
}

// COST OF OTHER CAR AHEAD SPEEDING UP AND SLOWING DOWN EGO CAR
double lane_speed_ahead(double ave_vel, double ego_vel) {
    double cost = 0.0;

    // if car ahead speed is way below optimal speed 
    // cost is higher since it's keeping the ego car
    if ((MAX_VEL - ave_vel) > 0.5)
        cost = (MAX_VEL - ave_vel) * STOP_COST/MAX_VEL;
    // car speed is +/- 0.5 of optimal speed
    if (fabs(MAX_VEL - ave_vel) < 0.5)
        cost = 0;
    // if car speed is above optimal speed but below speed limit
    if ((SPEED_LIMIT > ave_vel)  && ((ave_vel - MAX_VEL) > 0.5))
        cost = 0.2 * (ave_vel - MAX_VEL)/(SPEED_LIMIT - MAX_VEL) + STOP_COST;
    // if car speed is above the speed limit
    if (SPEED_LIMIT < ave_vel)
        cost = 1;
    // if car ahead speed is way different from ego speed 
    // cost is added since it's not keeping up with the traffic
    // if (fabs(ego_vel - ave_vel) > 0.5)
    //     cost += fabs(ego_vel - ave_vel)/ego_vel;

    return cost;
}


// COST OF SPEEDING UP AND SLOWING DOWN OF CARS BEHIND
vector <double> lanes_speed_behind(vector <double> &ave_vel, double ego_vel) {
    vector<double> costs;
    for (int i = 0; i < ave_vel.size(); i++) {
        double cost = lane_speed_behind(ave_vel[i], ego_vel);
        costs.push_back(cost);
    }
    return costs;
}

// COST OF OTHER CAR BEHIND SPEEDING UP AND SLOWING DOWN EGO CAR
double lane_speed_behind(double ave_vel, double ego_vel) {
    double cost = 0.0;

    // // if car behind speed is getting faster
    // // should cost more since it might hit the ego car
    // if ((MAX_VEL - ave_vel) > 0.5)
    //     // cost = (ave_vel * STOP_COST)/MAX_VEL;
    //     cost = 0;
    // // car speed is +/- 0.5 of optimal speed
    // // it can still hit ego car if ego is slow
    // if (fabs(MAX_VEL - ave_vel) < 0.5)
    //     // cost = STOP_COST;
    //     cost = (ave_vel * STOP_COST)/MAX_VEL;
    // // if car speed is above optimal speed but below speed limit
    // if ((SPEED_LIMIT > ave_vel)  && ((ave_vel - MAX_VEL) > 0.5))
    //     cost = 0.2 * (ave_vel - MAX_VEL)/(SPEED_LIMIT - MAX_VEL) + STOP_COST;
    // // if car speed is above the speed limit
    // if (SPEED_LIMIT < ave_vel)
    //     cost = 1;
    // // if car ahead speed is way different from ego speed 
    // // cost is added since it's not keeping up with the traffic
    // if (fabs(ego_vel - ave_vel) > 0.5)
    //     cost += fabs(ego_vel - ave_vel)/ego_vel;

    return cost;
}


// COST OF CLOSE DISTANCE TO OTHER CARS
vector <double> lanes_distance(vector <double> &nearest_s, double ego_dist) {
    vector <double> costs;
    for (int i = 0; i < nearest_s.size(); i++){
        double cost = lane_distance(nearest_s[i], ego_dist);
        costs.push_back(cost);
    }
    return costs;
}

// COST OF CLOSE DISTANCE TO A CAR
double lane_distance(double nearest_s, double ego_dist) {
    double cost = 0.0;

    double distance = fabs(ego_dist - nearest_s);
    // if car distance is way farther than buffer distance
    if (distance < DIST_BUFFER) cost = 1;

    return cost;
}



// COST OF EGO CAR
double ego_speed_cost(double ego_vel) {
    double cost = 0.0;

    // // car costs more if slow since it's not keeping up with the traffic
    // // car cost is only for the fraction it's not in optimal speed
    // if ((MAX_VEL - ego_vel) > 0.5)
    //     cost = (MAX_VEL - ego_vel)/MAX_VEL;
    //     // car speed is +/- 0.5 of optimal speed
    // if (fabs(MAX_VEL - ego_vel) < 0.5)
    //     cost = 0;
    // // if car speed is above optimal speed but below speed limit
    // if ((SPEED_LIMIT > ego_vel)  && ((ego_vel - MAX_VEL) > 0.5))
    //     cost = 0.2 * (ego_vel - MAX_VEL)/(SPEED_LIMIT - MAX_VEL) + STOP_COST;
    // // if car speed is above the speed limit
    // if (SPEED_LIMIT < ego_vel)
    //     cost = 1;

    return cost;
}


vector <double> calculate_COST (map<int, Vehicle> &vehicles, Vehicle &ego) 
{
    vector<double> vels_ahead;
    vector<double> dist_ahead;
    for (int i = 0; i < ego.lane_nearest_cars_ahead.size(); i++) {
        vels_ahead.push_back(vehicles[ego.lane_nearest_cars_ahead[i] ].v);
        dist_ahead.push_back(vehicles[ego.lane_nearest_cars_ahead[i] ].s);
    }

    // COST OF LANE SPEED - AHEAD
    vector <double> lanes_speed_cost_ahead = lanes_speed_ahead(vels_ahead, ego.v);

    // COST OF CLOSE DISTANCE - AHEAD
    vector <double> lanes_distance_cost_ahead = lanes_distance(dist_ahead, ego.s);
    

    vector<double> vels_behind;
    vector<double> dist_behind;
    for (int i = 0; i < ego.lane_nearest_cars_behind.size(); i++) {
        vels_behind.push_back(vehicles[ego.lane_nearest_cars_behind[i] ].v);
        dist_behind.push_back(vehicles[ego.lane_nearest_cars_behind[i] ].s);
    }
    
    // COST OF LANE SPEED - BEHIND
    vector <double> lanes_speed_cost_behind = lanes_speed_behind(vels_behind, ego.v);
   
    // COST OF CLOSE DISTANCE - BEHIND
    vector <double> lanes_distance_cost_behind = lanes_distance(dist_behind, ego.s);


    // CALCULATE TOTAL COST

    // calculate_cost_lanes_speed = lanes_speed_cost_ahead + lanes_speed_cost_behind
    vector <double> calculate_cost_lanes_speed;
    transform (lanes_speed_cost_ahead.begin(), lanes_speed_cost_ahead.end(), lanes_speed_cost_behind.begin(), back_inserter(calculate_cost_lanes_speed), plus<double>());
    
    // calculate_cost_lanes_dist = lanes_distance_cost_ahead + lanes_distance_cost_behind
    vector <double> calculate_cost_lanes_dist;
    transform (lanes_distance_cost_ahead.begin(), lanes_distance_cost_ahead.end(), lanes_distance_cost_behind.begin(), back_inserter(calculate_cost_lanes_dist), plus<double>());

    // calculate_cost = calculate_cost_lanes_speed + calculate_cost_lanes_dist
    vector <double> calculate_cost;
    transform (calculate_cost_lanes_speed.begin(), calculate_cost_lanes_speed.end(), calculate_cost_lanes_dist.begin(), back_inserter(calculate_cost), plus<double>());
    
    // calculate_cost = calculate_cost + ego vel
    for (double cost:calculate_cost) {
        cost += ego_speed_cost(ego.v);
    }

    cout << "calculate_cost size: " << calculate_cost.size() << endl; 
    cout << "Cost L0: " << calculate_cost[0] << "\t========================" << "Cost L1: " << calculate_cost[1] << "\t===============" << "Cost L2: " << calculate_cost[2] << "\t===================\n";
    
    return calculate_cost;

}













// COST OF NOT STAYING IN THE MIDDLE OF THE LANE
// LANE WIDTH IS PRESUMED TO BE ~ 4 METERS
// CAR WIDTH IS PRESUMED TO BE ~ 2 METERS
// THIS IS USED WHEN NOT CHANGING LANES
// double stay_in_lane_center(Vehicle &ego) {
//     double cost = 0.0;
//     // cost of going outside the lane

//     // cost depends on how far from the center of the lane is the car
//     if (fabs(ego.d - (4 * ego.lane + 2)) < 1){
//         cost = fabs((4 * ego.lane + 2) - ego.d) / 2;
//     } 

//     // swerving further to the left
//     if (ego.d <= (4 * ego.lane + 1)) {
//         cost = 1;
//     }

//     // swerving further to the right
//     if (ego.d >= (4 * ego.lane + 3)) {
//         cost = 1;
//     }
//     return  cost;
// }



// COST OF NOT STAYING IN THE MIDDLE OF THE LANE



// COST OF CHANGING LANE



// COST OF EXCEEDING THE 3 SECONDS LANE CHANGE





// double calculate_COST (Vehicle &possible_car_detected, Vehicle &ego, int new_LANE, double MAX_DIST, double MAX_VEL) 
// {

//     double stop_cost = 10.00;
//     double cost = 0.00;

//     // if slow vehicle is ahead of lane change gap
//     if ((possible_car_detected.s > ego.s) && ((possible_car_detected.s - ego.s) < MAX_DIST)) 
//     {

//  XXX   	// if it's fast, there's a low cost
// 	    // if it's slow, there's a great cost
//          if (possible_car_detected.v < ego.v) 
//         {
        
//             cost += stop_cost * (ego.v - possible_car_detected.v) / possible_car_detected.v;                   
        
//         }
//     }

//     // if vehicle is within lane change gap
//     if ((fabs(possible_car_detected.s - ego.s) <= (MAX_DIST * 2)) && (new_LANE != ego.lane)) 
//     {
    

//         cost += stop_cost;
    
//     }

//     // if fast vehicle is behind lane change gap
//  	if ((possible_car_detected.s < ego.s) && ((ego.s - possible_car_detected.s) < (MAX_DIST * 2))) 
// 	{
       
//  XXX      if (possible_car_detected.v > ego.v) 
//        {
       
//             cost += stop_cost * (possible_car_detected.v - ego.v) / ego.v;                          	
       
//        }

// 	}
    
//     // cost == 0 if there is no car in new lane or if car ahead in new lane is fast or car behind in new lane is slow

// 	return cost;
// }
// 		