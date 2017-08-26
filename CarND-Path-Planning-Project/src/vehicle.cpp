#include <iostream>
#include "vehicle.h"
#include "jmt.cpp"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(vector<double> config) {

    configure(config);
    state = "KL";
    //weights = {100, 10000, 10, 5000, 1};
    //cost_functions = {"target_speed", "collision", "traffic", "out_road", "change_lane"};
}

Vehicle::~Vehicle() {}

int Vehicle::get_lane(double d) {    
    int lane = d / lane_width;
    return lane;
}

int Vehicle::get_lane() {
    return get_lane(this->d);
}

void Vehicle::update_localization(double x, double y, 
    double s, double d, double yaw, double v) {

    this->x = x;
    this->y = x;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->v = v;
    this->lane = get_lane();
}

void Vehicle::configure(vector<double> config) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    lanes_available = config[0];
    lane_width = config[1];
    target_speed = config[2];
    max_acceleration = config[3];
    max_jerk = config[4];
    safe_distance = config[5];
    //time_interval = config[5];
}

void Vehicle::display() {
    cout << "s:    " << this->s << "\n";
    cout << "d:    " << this->d << "\n";
    cout << "lane: " << this->get_lane() << "\n";
    cout << "v:    " << this->v << "\n";
    cout << "a:    " << this->a << "\n";
    cout << "target v: " << this->target_speed << "\n";
    cout << "max a:    " << this->max_acceleration << "\n";
    cout << "max jerk: " << this->max_jerk << "\n";
}

void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions) {
    
    map<int, vector<vector<double>>>::iterator it = predictions.begin();
    vector<vector<vector<double>>> in_front;
    vector<vector<vector<double>>> left_front;
    vector<vector<vector<double>>> right_front;
    vector<vector<vector<double>>> left_collision;
    vector<vector<vector<double>>> right_collision;

    double next_s;
    double next_v = target_speed;

    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<double>> vec = it->second;
        //cout<< "Vehicle: " << v_id << " => " <<  vec[0][0] << "; " << vec[0][1] << endl;

        if((get_lane(vec[0][1]) == lane) && (vec[0][0] > s))
        {
            in_front.push_back(vec);
            
        } else if((get_lane(vec[0][1]) == lane+1) && (vec[0][0] > s - safe_distance/2.)) {

            if (vec[0][0] > s + safe_distance/2.) {
                right_front.push_back(vec);

            } else {
                right_collision.push_back(vec);
            }
        
        } else if((get_lane(vec[0][1]) == lane-1) && (vec[0][0] > s - safe_distance/2.)) {
        	if (vec[0][0] > s + safe_distance/2.) {
                left_front.push_back(vec);

            } else {
                left_collision.push_back(vec);
            }
        }

        it++;
    }
    
    cout<< "Left front: "<< left_front.size() << endl;
    cout<< "Left collision: "<< left_collision.size() << endl;
    cout<< "Center front: "<< in_front.size() << endl;
    cout<< "Right front: "<< right_front.size() << endl;
    cout<< "Right collision: "<< right_collision.size() << endl;

    double cost_center = 0;
    double leading_speed;
    double leading_s;

    if(in_front.size() > 0)
    {
    	double min_s = 1000;
    	vector<vector<double>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][1][0] - s) < min_s)
    		{
    			min_s = in_front[i][1][0] - s;
    			leading = in_front[i];
            }
            
            leading_speed = in_front[i][0][2];
            leading_s = in_front[i][0][0];
            cost_center += fabs(leading_speed - v) / pow((leading_s - s), 2);
    	}
        
        leading_s = leading[1][0];
        leading_speed = leading[1][2];
        double safe_s = leading_s - safe_distance;
        cout<< "Leading " << leading_s << "; " << safe_s << "; " << s << endl;
        double recovery_coeff = (safe_s - s <= 10 ? safe_s - s: 10);
        double delta_speed = fabs(v - leading_speed);

        if (s > safe_s) {
            
            if (v > leading_speed) {
                //v -= (.02 * max_acceleration);// * pow((s - safe_s), 2);
                v -= (delta_speed/safe_distance * (s - safe_s));
            }
            
            double cost_left = 10000000;
            double cost_right = 10000000;
            
            if (left_collision.size() == 0 && lane > 0) {
                cost_left = 0;
                if (left_front.size() > 0) {
                    for(int i = 0; i < left_front.size(); i++)
                    {
                        cost_left += fabs(left_front[i][0][2] - v) / pow((left_front[i][0][0] - s), 2);
                    }
                }
            }

            if (right_collision.size() == 0 && lane < lanes_available-1) {
                cost_right = 0;
                if (right_front.size() > 0) {
                    for(int i = 0; i < right_front.size(); i++)
                    {
                        cost_right += fabs(right_front[i][0][2] - v) / pow((right_front[i][0][0] - s), 2);
                    }
                }
            }

            if (cost_right < cost_center || cost_left < cost_center) {

                if (cost_right < cost_left) {
                    lane++;
                
                } else if (cost_left < cost_center) {
                    lane--;
                }
            }


        } else if (v <= target_speed) {
            v += (.01 * max_acceleration) * recovery_coeff;
        }

    } else if (v <= (target_speed)) {
        cout<< "Free lane!"<<endl;
        v += (.1 * max_acceleration);
    }
}