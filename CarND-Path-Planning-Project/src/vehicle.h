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

    int preferred_buffer = 6; // impacts "keep lane" behavior.

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

    vector<double> coeffs_s;
    vector<double> coeffs_d;

    double lanes_available;
    double lane_width;
    double target_speed;
    double max_acceleration;
    double max_jerk;
    //float time_interval;

    string state;

    vector<float> weights;
    vector<string> cost_functions;

    /**
  * Constructor
  */
    Vehicle(vector<double> config);

    /**
  * Destructor
  */
    virtual ~Vehicle();

    void update_state(map<int, vector<vector<double>>> predictions, float time_interval);

    void configure(vector<double> config);

    string display();

    //void increment(int dt);

    vector<double> state_at(float t);

    bool collides_with(Vehicle other, float at_time);

    collider will_collide_with(Vehicle other, int timesteps);

    void realize_state(map<int, vector<vector<double>>> predictions, float time_interval);

    void realize_constant_speed();

    //int _max_accel_for_lane(map<int, vector<vector<float>>> predictions, int lane, int s);

    void realize_keep_lane(map<int, vector<vector<double>>> predictions, float time_interval);

    void realize_lane_change(map<int, vector<vector<double>>> predictions, string direction);

    void realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction);

    vector<vector<double>> generate_predictions(float horizon);
    
    //vector<vector<double>> generate_trajectory(float wp_s);
    
    vector<string> successor_states();
    
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