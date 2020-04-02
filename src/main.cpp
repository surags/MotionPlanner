#include <uWS/uWS.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

enum LANE_STATE {
    STAY_LANE,
    TRANSITION_LANE
};

enum BRAKE_RATE {
    NO_BRAKE,
    LIGHT_BRAKE,
    MEDIUM_BRAKE,
    HARD_BRAKE
};

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Start in lane 1, which is the center lane (0 is left and 2 is right)
    int lane = 1;
    int target_lane = 1;
    
    LANE_STATE state = LANE_STATE::STAY_LANE;

    // Start at zero velocity and gradually accelerate
    double ref_vel = 0.0; // mph

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    h.onMessage([&map_waypoints_x, 
                    &map_waypoints_y, 
                    &map_waypoints_s,
                    &map_waypoints_dx, 
                    &map_waypoints_dy, 
                    &ref_vel, 
                    &lane, 
                    &target_lane,
                    &state]
                  (uWS::WebSocket<uWS::SERVER> ws, 
                    char *data, 
                    size_t length,
                    uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

        auto s = hasData(data);

        if (s != "") {
            auto j = json::parse(s);
            
            string event = j[0].get<string>();
            
            if (event == "telemetry") {
                // j[1] is the data JSON object
                
                // Main car's localization Data
                double car_x = j[1]["x"];
                double car_y = j[1]["y"];
                double car_s = j[1]["s"];
                double car_d = j[1]["d"];
                double car_yaw = j[1]["yaw"];
                double car_speed = j[1]["speed"];

                // Previous path data given to the Planner
                auto previous_path_x = j[1]["previous_path_x"];
                auto previous_path_y = j[1]["previous_path_y"];
                // Previous path's end s and d values 
                double end_path_s = j[1]["end_path_s"];
                double end_path_d = j[1]["end_path_d"];

                // Sensor Fusion Data, a list of all other cars on the same side 
                //   of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];

                json msgJson;

                /**
                 * TODO: define a path made up of (x,y) points that the car will visit
                 *   sequentially every .02 seconds
                 */

                int prev_size = previous_path_x.size();
                
                if (prev_size > 0) {
                    car_s = end_path_s;
                }
            
                bool too_close = false; // True if too close to a car in front
                bool too_close_left = false; // True if too close to a car on the left lane for lane switch
                bool too_close_right = false; // // True if too close to a car on the right lane for lane switch
                bool too_too_close = false;
            
                if(lane == 0) {
                    too_close_left = true; // No left lane to switch to
                } else if(lane == 2) { 
                    too_close_right = true; // No right lane to switch to
                }

                double left_dist = 1000000.0;
                double left_speed = 0.0;
                double right_dist = 1000000.0;
                double right_speed = 0.0;
                BRAKE_RATE brake_rate = BRAKE_RATE::NO_BRAKE;

                // Determine lane availability and ref_val
                for (int i = 0; i < sensor_fusion.size(); i++) {
                    // Check if the car is in the same lane as the ego vehicle
                    float d = sensor_fusion[i][6];

                    if( d < 0 ) {
                        // std::cout << "Opp dir" << std::endl;
                        continue;
                    }

                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    if(state == LANE_STATE::TRANSITION_LANE) {
                        // Car is in transition. Slow down for cars in front of me as opposed to cars in lane
                        if (d < (car_d+1.5) && d > (car_d-1.5)) {
                            // Car is in my partial-lane
                            // Calculate the check_car's future location
                            check_car_s += (double)prev_size * 0.02 * check_speed;
        
                            // If the check_car is within 30 meters in front, reduce ref_vel so that we don't hit it
                            if (check_car_s > car_s && (check_car_s - car_s) < 15) {
                                too_close = true;
                                brake_rate = BRAKE_RATE::MEDIUM_BRAKE;
                            }
                        }

                        // This is done to ensure car does not change to a new lane this iteration
                        too_close_left = true;
                        too_close_right = true;
                    } else {
                        if (d < (2+4*lane+2) && d > (2+4*lane-2)){
                            // Car is in my lane
                            // Calculate the check_car's future location
                            check_car_s += (double)prev_size * 0.02 * check_speed;

                            double dist = check_car_s - car_s;
                            // If the check_car is within 30 meters in front, reduce ref_vel so that we don't hit it
                            if (check_car_s > car_s && dist < 30) {
                                too_close = true;
                                brake_rate = std::max(brake_rate, BRAKE_RATE::LIGHT_BRAKE);
                                
                                if(dist < 20) {
                                    brake_rate = std::max(brake_rate, BRAKE_RATE::MEDIUM_BRAKE);
                                }

                                if(dist < 10) {
                                    brake_rate = std::max(brake_rate, BRAKE_RATE::HARD_BRAKE);
                                }
                            } 
                        } else if ( d < (2 + 4 * lane - 2) && d > (2 + 4 * (lane - 1) - 2) ) {
                            // Car is in the left lane
                            check_car_s += (double)prev_size * 0.02 * check_speed;
                            double dist = check_car_s - car_s;

                            // Find closest car in left lane
                            if(check_car_s > car_s && dist < left_dist) {
                                left_dist = dist;
                                left_speed = check_speed;
                            }

                            // If the check_car is within 35 meters in front, disqualify lane
                            if (check_car_s > car_s && (check_car_s - car_s) < 35) {
                                //ref_vel = 29.5;
                                too_close_left = true;
                            }

                            // If the check car is within 10 metres at back, disqualify lane (blindspot check)
                            if (check_car_s < car_s && (car_s - check_car_s) < 8) {
                                //ref_vel = 29.5;
                                too_close_left = true;
                            }

                        } else if ( d < (2 + 4 * (lane + 1) + 2) && d > (2+4*(lane) + 2) ) {
                            // Car is in the right lane
                            check_car_s += (double)prev_size * 0.02 * check_speed;

                            double dist = check_car_s - car_s;

                            // Find closest car in right lane
                            if(check_car_s > car_s && dist < right_dist) {
                                right_dist = dist;
                                right_speed = check_speed;
                            }
                            
                            // If the check_car is within 30 meters in front, disqualify lane
                            if (check_car_s > car_s && (check_car_s - car_s) < 35){
                                too_close_right = true;
                            }

                            // If the check car is within 10 metres at back, disqualify lane (blindspot check)
                            if (check_car_s < car_s && (car_s - check_car_s) < 8) {
                                too_close_right = true;
                            }
                        }
                    }

                }
            
                // Create a list of evenly spaced waypoints 30m apart
                // Interpolate those waypoints later with spline and fill it in with more points
                vector<double> ptsx;
                vector<double> ptsy;
            
                // Reference x, y, yaw states, either will be the starting point or end point of the previous path
                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);
            
                // if previous size is almost empty, use the car as starting reference
                if (prev_size < 2) {
                    // Use two points that make the path tangent to the car
                    double prev_car_x = car_x - 0.5 * cos(car_yaw);
                    double prev_car_y = car_y - 0.5 * sin(car_yaw);
                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_x);
                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_y);
                }
                // Use the previous path's end point as starting reference
                else {
                    ref_x = previous_path_x[prev_size-1];
                    ref_y = previous_path_y[prev_size-1];
                    double ref_x_prev = previous_path_x[prev_size-2];
                    double ref_y_prev = previous_path_y[prev_size-2];
                    ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
                    // Use the two points that make the path tangent to the previous path's end point
                    ptsx.push_back(ref_x_prev);
                    ptsx.push_back(ref_x);
                    ptsy.push_back(ref_y_prev);
                    ptsy.push_back(ref_y);
                }

                vector<double> next_wp0;
                vector<double> next_wp1;
                vector<double> next_wp2;
                bool goStraight = true; 
                
                if(state == LANE_STATE::TRANSITION_LANE) {
                    // Check if lane transition is complete
                    if (car_d < (2+4*target_lane+1) && car_d > (2+4*target_lane-1)) {
                        // Car within +/- 1 d of center of lane. Lane change complete!
                        lane = target_lane;
                        state = LANE_STATE::STAY_LANE;
                        std::cout << "State change: STAY LANE; Lane " << lane << std::endl; 
                    }

                }

                if(state == LANE_STATE::STAY_LANE && too_close) {
                    // Check lane switch possibility
                    if(lane == 0) {
                        // Can only move right
                        if(!too_close_right) {
                            // Change to right lane
                            target_lane = lane + 1;
                            goStraight = false;
                            state = LANE_STATE::TRANSITION_LANE;
                            std::cout << "State change: TRANSITION_LANE " << lane << " -> " << target_lane << std::endl;
                        } 
                    } else if (lane == 1) {
                        bool check_done = false;
                        // Check both left and right
                        if(!too_close_left && !too_close_right) {
                            // Since both lanes are available chose the one with the furthest car
                            if(left_dist > right_dist) {
                                // Change to left lane
                                target_lane = lane - 1;
                                goStraight = false;
                                state = LANE_STATE::TRANSITION_LANE;
                                std::cout << "State change: TRANSITION_LANE " << lane << " -> " << target_lane << std::endl;
                            } else {
                                // Change to right lane
                                target_lane = lane + 1;
                                goStraight = false;
                                state = LANE_STATE::TRANSITION_LANE;
                                std::cout << "State change: TRANSITION_LANE " << lane << " -> " << target_lane << std::endl;    
                            }
                        } else if(!too_close_left) {
                            // Change to left lane
                            target_lane = lane - 1;
                            goStraight = false;
                            state = LANE_STATE::TRANSITION_LANE;
                            std::cout << "State change: TRANSITION_LANE " << lane << " -> " << target_lane << std::endl;

                        } else if (!too_close_right) {
                            // Change to right lane
                            target_lane = lane + 1;
                            goStraight = false;
                            state = LANE_STATE::TRANSITION_LANE;
                            std::cout << "State change: TRANSITION_LANE " << lane << " -> " << target_lane << std::endl;
                        }
                    
                    } else if (lane == 2) {
                        // Can only go left
                        if(!too_close_left) {
                            // Change to left lane
                            target_lane = lane - 1;
                            goStraight = false;
                            state = LANE_STATE::TRANSITION_LANE;
                            std::cout << "State change: TRANSITION_LANE " << lane << " -> " << target_lane << std::endl;
                        }
                    }
                }

                // Add evenly 30m spaced points ahead of the starting reference
                // vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                // vector<double> next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                // vector<double> next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                if(state == LANE_STATE::TRANSITION_LANE) {
                    // next_wp0 = getXY(car_s+30, (2 + 4*target_lane + car_d) / 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    // std::cout << "Target (s, d): " << "(" << car_s + 30 << "," << (2 + 4*target_lane + car_d) / 2 << ")" << std::endl;
                    next_wp0 = getXY(car_s+30, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    next_wp1 = getXY(car_s+60, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    next_wp2 = getXY(car_s+90, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                } else {
                    next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    next_wp1 = getXY(car_s+60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    next_wp2 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                }

                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);
                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);
            
                for (int i = 0; i < ptsx.size(); i++) {
                    // shift car reference angle to 0 degrees
                    double shift_x = ptsx[i]-ref_x;
                    double shift_y = ptsy[i]-ref_y;
                    ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
                    ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
                }
            
                // Create a spline
                tk::spline s;
                // Set (x,y) points to the spline (i.e. fits a spline to those points)
                s.set_points(ptsx, ptsy);
                
                // Define the actual (x,y) points we will use for the planner
                vector<double> next_x_vals;
                vector<double> next_y_vals;

                // Start with all of the previous path points from last time
                for (int i = 0; i < previous_path_x.size(); i++) {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }

                // Calculate how to break up spline points so that we travel at desired velocity
                double target_x = 30.0; // 30.0 m is the distance horizon
                double target_y = s(target_x);
                // std::cout << "target_y:" << target_y << std::endl; 
                double target_dist = sqrt(target_x*target_x + target_y*target_y); // this is the d in the diagram
                double x_add_on = 0.0; // Related to the transformation (starting at zero)

                // Fill up the rest of path planner after filling it with previous points, will always output 50 points
                for (int i = 1; i <= 50-previous_path_x.size(); i++) {
                    if (too_close) {
                        switch(brake_rate) {
                            case BRAKE_RATE::LIGHT_BRAKE:
                                ref_vel -= .056;
                                break;
                            case BRAKE_RATE::MEDIUM_BRAKE:
                                ref_vel -= .112;
                                break;
                            case BRAKE_RATE::HARD_BRAKE:
                                ref_vel -= .224;
                                break;
                            default:
                                ref_vel -= .224;
                                break;
                        }
                        // ref_vel -= .112;
                    } else if (ref_vel < 49.5) {
                        ref_vel += .224;
                    }

                    double N = (target_dist/(0.02*ref_vel/2.24));
                    double x_point = x_add_on + target_x/N;
                    double y_point = s(x_point);
                    
                    x_add_on = x_point;
                    
                    double x_ref = x_point;
                    double y_ref = y_point;
                    
                    // Rotate x, y back to normal
                    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
                    
                    x_point += ref_x;
                    y_point += ref_y;
                    
                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);
                }

                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

                auto msg = "42[\"control\","+ msgJson.dump()+"]";

                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
        } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    }  // end websocket if
    }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}