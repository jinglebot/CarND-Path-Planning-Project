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
vector <double> lanes_speed_behind(vector <double> &ave_vel, double ego_vel, int lane) {
    vector<double> costs;
    for (int i = 0; i < ave_vel.size(); i++) {
        double cost = 0;
        if (lane != i)
            cost = lane_speed_behind(ave_vel[i], ego_vel);
        costs.push_back(cost);
    }
    return costs;
}

// COST OF OTHER CAR BEHIND SPEEDING UP AND SLOWING DOWN EGO CAR
double lane_speed_behind(double ave_vel, double ego_vel) {
    double cost = 0.0;
    cost = (ave_vel * STOP_COST)/MAX_VEL;
    return cost;
}

// COST OF FUTURE POSITION OF CARS BEHIND 
vector <double> cars_future_pos(vector <double> &ave_vel, vector <double> &nearest_s, Vehicle &ego) {
    vector <double> costs;
    for (int i = 0; i < ave_vel.size(); i++) {
        double cost = car_future_pos(ave_vel[i], nearest_s[i], ego);
        costs.push_back(cost);
    }
    return costs;
}

// COST OF FUTURE POSITION OF CAR BEHIND 
double car_future_pos(double ave_vel, double nearest_s, Vehicle &ego) {
    double cost = 0.0;
    double future_s = ego.get_future_car_position(ave_vel, nearest_s);
    if (future_s > (ego.s - DIST_BUFFER) ) {
        cost = 1;
    }
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


// COST OF LANE CHANGE
vector <double> goal_distance(map<int, Vehicle> &vehicles, Vehicle &ego) {
    vector<double> costs (NUM_LANES, 0.0);

    int next_lane_left = ego.lane - 1;
    double cost_left = 0.0;
    if (next_lane_left >= 0) {
        double delta_d_left = fabs(ego.d - (4 * next_lane_left + 2) );
        double distance_to_goal_left = fabs(vehicles[ego.lane_nearest_cars_ahead[next_lane_left] ].s - DIST_BUFFER - ego.s);
        cost_left += 1 - exp( (-1 * delta_d_left) / distance_to_goal_left );
    }
    costs[next_lane_left] = cost_left;

    int next_lane_right = ego.lane + 1;
    double cost_right = 0.0;
    if (next_lane_right <= (NUM_LANES - 1) ) {
        double delta_d_right = fabs(ego.d - (4 * next_lane_right + 2) );
        double distance_to_goal_right = fabs(vehicles[ego.lane_nearest_cars_ahead[next_lane_right] ].s - DIST_BUFFER - ego.s);
        cost_right += 1 - exp( (-1 * delta_d_right) / distance_to_goal_right );
    }
    costs[next_lane_right] = cost_right;

    return costs;
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
    vector <double> lanes_speed_cost_behind = lanes_speed_behind(vels_behind, ego.v, ego.lane);
   
    // COST OF CLOSE DISTANCE - BEHIND
    vector <double> lanes_distance_cost_behind = lanes_distance(dist_behind, ego.s);

    vector <double> future_pos = cars_future_pos(vels_behind, dist_behind, ego);

    vector <double> distance_to_goal = goal_distance(vehicles, ego);



    // CALCULATE TOTAL COST

    // calculate_cost_lanes_speed = lanes_speed_cost_ahead + lanes_speed_cost_behind
    vector <double> calculate_cost_lanes_speed;
    if (ego.state != "KL") 
        transform (lanes_speed_cost_ahead.begin(), lanes_speed_cost_ahead.end(), lanes_speed_cost_behind.begin(), back_inserter(calculate_cost_lanes_speed), plus<double>());
    else
        calculate_cost_lanes_speed = lanes_speed_cost_ahead;

    // calculate_cost_lanes_dist = lanes_distance_cost_ahead + lanes_distance_cost_behind
    vector <double> calculate_cost_lanes_dist;
    transform (lanes_distance_cost_ahead.begin(), lanes_distance_cost_ahead.end(), lanes_distance_cost_behind.begin(), back_inserter(calculate_cost_lanes_dist), plus<double>());

    // calculate_cost = calculate_cost_lanes_speed + calculate_cost_lanes_dist
    vector <double> calculate_cost_lanes;
    transform (calculate_cost_lanes_speed.begin(), calculate_cost_lanes_speed.end(), calculate_cost_lanes_dist.begin(), back_inserter(calculate_cost_lanes), plus<double>());
    
    // calculate_cost_lanes_dist = lanes_distance_lanes + future positions of other vehicles
    vector <double> calculate_cost_future_ego;
    transform (future_pos.begin(), future_pos.end(), distance_to_goal.begin(), back_inserter(calculate_cost_future_ego), plus<double>());

    // calculate_cost_lanes_dist = lanes_distance_lanes + future positions of other vehicles
    vector <double> calculate_cost;
    transform (calculate_cost_lanes.begin(), calculate_cost_lanes.end(), calculate_cost_future_ego.begin(), back_inserter(calculate_cost), plus<double>());

    // calculate_cost = calculate_cost + ego vel
    for (double cost:calculate_cost) {
        cost += ego_speed_cost(ego.v);
    }

    // cout << "calculate_cost size: " << calculate_cost.size() << endl; 
    cout << "===== Goal Cost L0: " << distance_to_goal[0] << "\t=================== Goal Cost L1: " << distance_to_goal[1] << "\t=============== Goal Cost L2: " << distance_to_goal[2] << "\t===================\n";
    cout << "========== Cost L0: " << calculate_cost[0] << "\t=================== Cost L1: " << calculate_cost[1] << "\t=============== Cost L2: " << calculate_cost[2] << "\t===================\n";
    
    return calculate_cost;

}

