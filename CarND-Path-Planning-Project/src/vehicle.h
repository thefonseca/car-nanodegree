#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle
{
  public:
    struct collider
    {
        bool collision; // is there a collision?
        int time;       // time collision happens
    };

    int L = 1;

    double safe_distance;

    float lane;
    double x;
    double y;
    double s;
    double d;
    double vx;
    double vy;
    double v;
    double yaw;
    double a;
    double last_lane_change_s;

    double lanes_available;
    double lane_width;
    double target_speed;
    double max_acceleration;
    double max_jerk;
    //float time_interval;

    string state;

    vector<vector<vector<double>>> in_front;
    vector<vector<vector<double>>> left_front;
    vector<vector<vector<double>>> right_front;
    vector<vector<vector<double>>> left_collision;
    vector<vector<vector<double>>> right_collision;

    /**
  * Constructor
  */
    Vehicle(vector<double> config);

    /**
  * Destructor
  */
    virtual ~Vehicle();

    void update_state(map<int, vector<vector<double>>> predictions);
    void process_sensor_data(map<int, vector<vector<double>>> predictions);

    void configure(vector<double> config);

    void display();

    void realize_keep_lane(map<int, vector<vector<double>>> predictions);

    float get_cost(string cost_function, 
      vector<vector<double>> trajectory_for_state, 
      map<int, vector<vector<double>>> predictions);

    Vehicle clone();
    
    void update_localization(double x, double y, 
      double s, double d, double yaw, double v);
    
    int get_lane();
    int get_lane(double d);
};

#endif