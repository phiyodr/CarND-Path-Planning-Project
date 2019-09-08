/*
Lane changes 
*/

#include <uWS/uWS.h>
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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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

  //new: 2line

  int lane = 1;
  double ref_vel = 0.;

  h.onMessage([&lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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


          int prev_size = previous_path_x.size();

          /*
          START: for part 3
          */
          
          if (prev_size > 0) {
              car_s = end_path_s;
          }

          bool too_close = false;
          double closest_car_left = 100000; 
          double closest_car_right = 100000;  

          for (int i = 0; i < sensor_fusion.size(); i++){
              float d = sensor_fusion[i][6];
              int other_car_lane;
              if (d < 0) {
                other_car_lane = -99;
              } else if (d > 0 && d <= 4) {
                other_car_lane = 0;
              } else if (d > 4 && d <= 8) {
                other_car_lane = 1;
              } else if (d > 8 && d <= 12) {
                other_car_lane = 2;
              } else if (d > 12) {
                other_car_lane = 99;
              }

              if (fabs(other_car_lane) >= 99)
                continue;


              //if ( d < (2+4*lane+2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
                
              double prev_size_f =  static_cast<double>(prev_size);
              check_car_s +=prev_size_f*0.02*check_speed;


              //if ((check_car_s > car_s) && (check_car_s - car_s) < 30){
              if (lane - other_car_lane == 0) { 
                // Do some logic here, 
                //ref_vel = 29.5;
                if (check_car_s > car_s && check_car_s - car_s < 30){
                  too_close = true;
                } 
              } else if (lane - other_car_lane == 1) { 
                if (check_car_s - car_s < closest_car_left &&
                  check_car_s - car_s > -0.75 * 30){
                  closest_car_left = check_car_s - car_s;
                  //closest_car_left_lane_abs = fabs(closest_car_left);
                }
              } else if (lane - other_car_lane == -1 ) {
                if (check_car_s - car_s < closest_car_right  &&
                    check_car_s - car_s > -0.75 * 30){
                  closest_car_right = check_car_s - car_s;
                  //closest_car_left_lane_abs = fabs(closest_car_left_lane)

                }
              }
              else{
                continue ; // Ignore cars two lane apart from the ego car.
              }
                /*
                START: for part 5
                */
                //if (lane >0){
                //  lane=0;
                //}
                
                /*
                End: for part 5
                */

            } // end i for each sensor_fusion=car
              //}
          /*
          START: for part 4... note also at the beginning: ref_vel set to 0
          */
          //if (too_close){
          //  ref_vel -= .224;
          //}
          //else if (ref_vel < 49.5){
          //  ref_vel += .224;
          //}
          if (too_close) { // Car ahead
            if (closest_car_left > 30 && closest_car_left < -0.75*30 &&
                (closest_car_left > closest_car_right || lane == 2) &&
                lane != 0) {
              lane--; // Change lane left.
            } else if (closest_car_right > 30 && closest_car_left < -0.75*30 &&
                 (closest_car_right > closest_car_left || lane == 0) &&
                 lane != 2) {
              lane++; // Change lane right.
            } else {
              ref_vel -= .224; // Adjust speed with the speed of the vehicle in lane.
            }
          } else {
            if (ref_vel < 49.5){
              ref_vel += .224;
            }
          }
          /*
          END: for part 4
          */

          /*
          END: for part 3
          */
          //new begin
          vector<double> ptsx;
          vector<double> ptsy;

          // ref x,y,yaw
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); 
          

          if (prev_size <2){

            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
          
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else { //use the prev path's end point as starting ref
            //Redefine ref state as prev path end point

            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            //Use 2 points that make the path tangent to the prev. path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          //in frenet add evenly 30m spaced points ahead of staring ref

        vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);


        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);

        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]); // now we have 5 points: 2 from loop above and 3 from now above

        for (int i=0; i < ptsx.size(); i++){
          // shift car ref angle to 0 deg
          double shift_x = ptsx[i] - ref_x;
          double shift_y = ptsy[i] - ref_y;

          ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw);
          ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw);


        } // end for i

        tk::spline s;
        s.set_points(ptsx, ptsy);

        //define the acutal x,y points we will use for the planner
        vector<double> next_x_vals;
        vector<double> next_y_vals;

        // Start with all the prev. path points fomr last time
        for (int i = 0; i < previous_path_x.size(); i++){
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
        }

        // Calc how to break up spline points so that we travel at our desired ref velocity
        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

        double x_add_on = 0; //start at origin

        // Fill up the rest of our path okanner after filling it with the prev. points, here we will alwys output 50 points
        for (int i = 1; i <= 50 - previous_path_x.size(); i++){
          double N = (target_dist / (.02*ref_vel/2.24)); //nb of hash marks
          double x_point = x_add_on + (target_x)/N;
          double y_point = s(x_point);

          x_add_on = x_point;

          double x_ref = x_point;
          double y_ref = y_point;

          // rotate back to normal adter rotatnug it earlier
          x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
          y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

          x_point += ref_x;
          y_point += ref_y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);

        } // end loop i



          // end new
          json msgJson;

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