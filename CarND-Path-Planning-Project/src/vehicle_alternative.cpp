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
    weights = {100, 10000, 10, 5000, 1};
    cost_functions = {"target_speed", "collision", "traffic", "out_road", "change_lane"};
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

vector<string> Vehicle::successor_states() {
   
	/*
    Given a state, return candidates for successor states.
    */
    string state = this->state;

    // testing...
    return {"KL"};
    
    if(state.compare("CS") == 0)
    {
    	return {"KL", "CS"};
    }
    else if(state.compare("KL") == 0)
    {
        if(lane == 0) {
            //return {"PLCL", "KL"};
            return {"LCR", "KL"};
        }
        
        if(lane == lanes_available-1) {
            //return {"PLCR", "KL"};
            return {"LCL", "KL"};
        }
        
        //return {"PLCL", "PLCR", "KL"};
        return {"LCL", "LCR", "KL"};
    }
    else if(state.compare("LCL") == 0)
    {
        /*if(lane == 0) {
            return {"PLCL", "KL"};
        }
        
        if(lane == lanes_available-1) {
            return {"PLCR", "KL"};
        }
        
    	return {"PLCL", "PLCR", "KL"};*/
    	return {"KL"};
    }
    else if(state.compare("LCR") == 0)
    {
    	/*if(lane == 0) {
            return {"PLCL", "KL"};
        }
        
        if(lane == lanes_available-1) {
            return {"PLCR", "KL"};
        }
        
    	return {"PLCL", "PLCR", "KL"};*/
    	return {"KL"};
    }
    else if(state.compare("PLCL") == 0)
    {
    	return {"KL", "LCL"};
    }
    else if(state.compare("PLCR") == 0)
    {
    	return {"KL", "LCR"};
    }

    return {};
}


float Vehicle::get_cost(string cost_function, vector<vector<double>> trajectory_for_state, 
    map<int,vector<vector<double>>> predictions) {

    float cost = 0;
   
    if (cost_function.compare("target_speed") == 0) {
        
        for (int t=0; t < trajectory_for_state.size(); t++) {
        
            //std::cout<<"trajectory " << t << "size: " << trajectory_for_state.size() << endl;
            
            double s_ = trajectory_for_state[t][0];
            double d_ = trajectory_for_state[t][1];
            //double v_ = trajectory_for_state[t][2];
            
            map<int, vector<vector<double>>>::iterator it = predictions.begin();
            vector<vector<vector<double>>> in_front;
            
            while(it != predictions.end()) {

                int v_id = it->first;
                vector<vector<double>> vec = it->second;

                if (t < vec.size()) {
                    if((get_lane(vec[t][1]) == get_lane(d_)) && vec[t][0] > s_) {
                        in_front.push_back(vec);
                    }
                }
        
                it++;
            }
            
            double leading_speed = target_speed;
            //double leading_s = s + 100000;

            if(in_front.size() > 0) {

                double min_s = safe_distance;
                vector<vector<double>> leading = {};
                for(int i = 0; i < in_front.size(); i++)
                {
                    if (t < in_front[i].size()) {
                        if((in_front[i][t][0] - s_) < min_s)
                        {
                            min_s = in_front[i][t][0] - s_;
                            leading = in_front[i];
                            //leading_s = leading[t][0];
                            leading_speed = leading[t][2];
                        }
                    }
                }
            }

            //cost += fabs(leading_speed - target_speed) / pow((leading_s - s_), 2);
            cost += (fabs(leading_speed - target_speed) / target_speed);
        }
    }

    if (cost_function.compare("traffic") == 0) {
        
        for (int t=0; t < trajectory_for_state.size(); t++) {

            double s_ = trajectory_for_state[t][0];
            double d_ = trajectory_for_state[t][1];
            //double v_ = trajectory_for_state[t][2];
            
            map<int, vector<vector<double>>>::iterator it = predictions.begin();
            vector<vector<vector<double>>> in_front;
            
            while(it != predictions.end())
            {
                int v_id = it->first;
                vector<vector<double>> vec = it->second;
                
                if (t < vec.size()) {
                    if((get_lane(vec[t][1]) == get_lane(d_)) && vec[t][0] > s_) {
                        in_front.push_back(vec);
                    }
                }

                it++;
            }

            double cost_trajectory = 0;
            
            if (in_front.size() > 0) {
                for(int i = 0; i < in_front.size(); i++) {
                    //cost += fabs(in_front[i][0][2] - v) / pow((in_front[i][0][0] - s), 2);
                    if (t < in_front[i].size()) {

                        //cout<< "in front: " << in_front.size() << endl;
                        //cout<< "lane: " << get_lane(d_) << "; vehicle lane: " << get_lane(in_front[i][t][1]) << "; vehicle speed: " << in_front[i][t][2] << "; target speed: " << target_speed << endl;
                        cost_trajectory += (fabs(in_front[i][t][2] - target_speed) / target_speed);
                    }
                }

                cost_trajectory /= in_front.size();
            }

            cost += cost_trajectory;
        }
    }

    if (cost_function.compare("collision") == 0) {
        
        
        for (int t=0; t < trajectory_for_state.size(); t++) {
            
            double s_ = trajectory_for_state[t][0];
            double d_ = trajectory_for_state[t][1];
            
            map<int, vector<vector<double>>>::iterator it = predictions.begin();
            vector<vector<vector<double>>> center;
            vector<vector<vector<double>>> center_collision;
            
            while(it != predictions.end())
            {
                int v_id = it->first;
                vector<vector<double>> vec = it->second;
                //cout<< "Vehicle: " << v_id << " => " <<  vec[0][0] << "; " << vec[0][1] << endl;
        
                if (t < vec.size()) {
                    if((get_lane(vec[t][1]) == get_lane(d_)) && (vec[t][0] > s_))
                    {
                        if ((vec[t][0] < s_ + safe_distance/2.) && (vec[t][0] > s_ - safe_distance/2.)) {
                            center_collision.push_back(vec);
                        }

                        center.push_back(vec);
                    }
                }
                
                it++;
            }

            if (center.size()) {
                cost += (center_collision.size() / center.size());
            }
        }

    }

    if (cost_function.compare("out_road") == 0) {
        
        for (int t=0; t < trajectory_for_state.size(); t++) {
            
            double s_ = trajectory_for_state[t][0];
            double d_ = trajectory_for_state[t][1];
            int lane = get_lane(d_);

            if(lane < 0 || lane >= lanes_available) {
                cost++;
            }
        }
    }

    if (cost_function.compare("change_lane") == 0) {

        for (int t=0; t < trajectory_for_state.size(); t++) {
            
            if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
                cost++;
            } 
        }
    }

    cost /= trajectory_for_state.size();
    std::cout<< "Cost function '" << cost_function << "' for state '" << state << "' = " << cost << endl;
    return cost;
}


// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<double>>> predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    //state = "KL"; // this is an example of how you change state.

    vector<string> possible_successor_states = successor_states();
    //std::cout<< possible_successor_states.size() << "\n";

    // keep track of the total cost of each state.
    vector<float> costs;
    
    for (int i=0; i < possible_successor_states.size(); i++) {
        
        // generate a rough idea of what trajectory we would
        // follow IF we chose this state.
        state = possible_successor_states[i];
        std::cout<<"Candidate state: " << state <<"\n";
        Vehicle sim_v = clone();
        //cout<< "Vehicle before: "<< endl;
        //sim_v.display();
        sim_v.realize_state(predictions, true);
        //cout<< "Vehicle after: "<< endl;
        //sim_v.display();
        //std::cout<<"Generating predictions..." << endl;
        vector<vector<double>> trajectory_for_state = sim_v.generate_predictions(10);
        //vector<vector<float>> trajectory_for_state;
        //trajectory_for_state.push_back(sim_v.state_at(time_interval));
        
        // calculate the "cost" associated with that trajectory.
        float cost_for_state = 0;
        for (int c=0; c < cost_functions.size(); c++) {
            // apply each cost function to the generated trajectory
            float cost_for_cost_function = sim_v.get_cost(cost_functions[c], trajectory_for_state, predictions);

            // multiply the cost by the associated weight
            cost_for_state += weights[c] * cost_for_cost_function;
        }
        
        //costs.append({'state' : state, 'cost' : cost_for_state})
        costs.push_back(cost_for_state);
    }
    
    // Find the minimum cost state.
    string best_next_state;
    float min_cost = 9999999;
    
    for (int i=0; i < possible_successor_states.size(); i++) {
        if (costs[i] < min_cost){
            min_cost = costs[i];
            best_next_state = possible_successor_states[i]; 
        }
    }
    
    std::cout<< "Next state: " << best_next_state << "; cost: " << min_cost << "\n";
    state = best_next_state;
    realize_state(predictions, false);
}

Vehicle Vehicle::clone() {
    vector<double> config = {lanes_available, lane_width, 
        target_speed, max_acceleration, max_jerk,
        safe_distance
    };
    Vehicle clone = Vehicle(config);
    clone.state = state;

    clone.update_localization(x, y, s, d, yaw, v);
    return clone;
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

	//ostringstream oss;
	
    cout << "s:    " << this->s << "\n";
    cout << "d:    " << this->d << "\n";
    cout << "lane: " << this->get_lane() << "\n";
    cout << "v:    " << this->v << "\n";
    cout << "a:    " << this->a << "\n";
    cout << "target v: " << this->target_speed << "\n";
    cout << "max a:    " << this->max_acceleration << "\n";
    cout << "max jerk: " << this->max_jerk << "\n";
    
    //return oss.str();
}

/*void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}*/

vector<double> Vehicle::state_at(double t) {
	/*
    Predicts state of vehicle in t seconds
    */
    float s_ = this->s + this->v * t; // + this->a * t * t / 2;
    float v_ = this->v;// + this->a * t;
    return {s_, d, v_};
}

bool Vehicle::collides_with(Vehicle other, float at_time) {

	/*
    Simple collision detection.
    */
    vector<double> check1 = state_at(at_time);
    vector<double> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<double>>> predictions, bool simulated) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L", simulated);
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R", simulated);
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }
}

void Vehicle::realize_constant_speed() {
	a = 0;
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
    		if((in_front[i][0][0] - s) < min_s)
    		{
    			min_s = in_front[i][0][0]-s;
    			leading = in_front[i];
            }
            
            leading_speed = in_front[i][0][2];
            leading_s = in_front[i][0][0];
            cost_center += fabs(leading_speed - v) / pow((leading_s - s), 2);
    	}
        
        leading_s = leading[0][0];
        leading_speed = leading[0][2];
        double safe_s = leading_s - safe_distance;
        cout<< "Leading " << leading_s << "; " << safe_s << "; " << s << endl;
        double recovery_coeff = (safe_s - s <= 10 ? safe_s - s: 10);
        double delta_speed = fabs(v - leading_speed);

        if (s > safe_s) {
            
            //v -= (.02 * max_acceleration);// * pow((s - safe_s), 2);

            if (v > leading_speed) {
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
            //v += (delta_speed/safe_distance * (s - safe_s));
        }

    } else if (v <= (target_speed)) {
        cout<< "Free lane!"<<endl;
        v += (.1 * max_acceleration);
    }
}

/*void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions) {
    
    map<int, vector<vector<double>>>::iterator it = predictions.begin();
    vector<vector<vector<double>>> in_front;
    vector<vector<vector<double>>> left_front;
    vector<vector<vector<double>>> right_front;
    vector<vector<vector<double>>> left_collision;
    vector<vector<vector<double>>> right_collision;

    //double next_s;
    //double next_v = target_speed;

    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<double>> vec = it->second;
        //cout<< "Vehicle: " << v_id << " => " <<  vec[0][0] << "; " << vec[0][1] << endl;

        if((get_lane(vec[0][1]) == get_lane(d)) && (vec[0][0] > s))
        {
            in_front.push_back(vec);
        }

        it++;
    }
    
    //cout<< "Left front: "<< left_front.size() << endl;
    //cout<< "Left collision: "<< left_collision.size() << endl;
    cout<< "Center front: "<< in_front.size() << endl;
    //cout<< "Right front: "<< right_front.size() << endl;
    //cout<< "Right collision: "<< right_collision.size() << endl;

    //double cost_center = 0;
    double leading_speed;
    double leading_s;

    if(in_front.size() > 0)
    {
    	double min_s = 10000; //5. * safe_distance;
        vector<vector<double>> leading = {};
        
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][0] - s) < min_s)
    		{
    			min_s = in_front[i][0][0] - s;
                leading = in_front[i];
            }
        }
        
        leading_s = leading[0][0];
        leading_speed = leading[0][2];

        double safe_s = leading_s - safe_distance;
        cout<< "Leading " << leading_s << "; " << safe_s << "; " << s << endl;
        double recovery_coeff = (safe_s - s <= 10 ? safe_s - s: 10);
        double delta_speed = fabs(v - leading_speed);

        if (s > safe_s) {
            
            //v -= (.02 * max_acceleration);// * pow((s - safe_s), 2);
            if (v > leading_speed) {
                v -= (delta_speed/safe_distance * (s - safe_s));
            }
        
        } else if (v <= target_speed) {
            v += (.01 * max_acceleration) * recovery_coeff;
        }

    } else if (v <= (target_speed)) {
        cout<< "Free lane!"<<endl;
        v += (.1 * max_acceleration);
    }
}*/

void Vehicle::realize_lane_change(map<int, vector<vector<double>>> predictions, string direction, bool simulated) {
    
    if (direction.compare("L") == 0) {
        lane--;
        if (simulated) {
            d -= lane_width;
        }

    
    } else {
        lane++;
        if (simulated) {
            d += lane_width;
        }
    }
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<double>>>::iterator it = predictions.begin();
    vector<vector<vector<double>>> at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<double> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	double max_s = -1000;
    	vector<vector<double>> nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	double target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	double delta_v = this->v - target_vel;
    	double delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		float my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<double>> Vehicle::generate_predictions(float horizon = 10) {

	vector<vector<double>> predictions;
    for(double i = 0; i < horizon; i++)
    {
      vector<double> check1 = state_at(i);
      //cout<< "prediction " << i << endl;
      predictions.push_back(state_at(i));
  	}
    return predictions;

}