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
    weights = {1.};
    cost_functions = {"target_speed"};
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

    if(coeffs_s.size() == 0) {
        coeffs_s = {s, v, 0, 0, 0, 0};
    }

    if(coeffs_d.size() == 0) {
        coeffs_d = {d, 0, 0, 0, 0, 0};
    }
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
            return {"PLCL", "KL"};
        }
        
        if(lane == lanes_available-1) {
            return {"PLCR", "KL"};
        }
        
        return {"PLCL", "PLCR", "KL"};
    }
    else if(state.compare("LCL") == 0)
    {
        if(lane == 0) {
            return {"PLCL", "KL"};
        }
        
        if(lane == lanes_available-1) {
            return {"PLCR", "KL"};
        }
        
    	return {"PLCL", "PLCR", "KL"};
    	//return {"KL"};
    }
    else if(state.compare("LCR") == 0)
    {
    	if(lane == 0) {
            return {"PLCL", "KL"};
        }
        
        if(lane == lanes_available-1) {
            return {"PLCR", "KL"};
        }
        
    	return {"PLCL", "PLCR", "KL"};
    	//return {"KL"};
    }
    else if(state.compare("PLCL") == 0)
    {
    	return {"KL", "LCL"};
    	//return {"LCL"};
    }
    else if(state.compare("PLCR") == 0)
    {
    	return {"KL", "LCR"};
    	//return {"LCR"};
    }
}


float Vehicle::get_cost(string cost_function, vector<vector<double>> trajectory_for_state, 
    map<int,vector<vector<double>>> predictions) {
   
   if(cost_function.compare("target_speed") == 0) {
       float cost = 0;
        
       for (int t=0; t < trajectory_for_state.size(); t++) {
           int v_ = trajectory_for_state[t][2];
           
           float dv = abs(target_speed - v_)/target_speed;
           cost += dv;
       }

       cost /= trajectory_for_state.size();
       
       std::cout<< "Cost: " << cost<<"\n";
       return cost;
   }
    
   return 0.;
}


// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<double>>> predictions, float time_interval) {
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
        sim_v.realize_state(predictions, time_interval);
        std::cout<<"Generating predictions..." << endl;
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
    
    std::cout<< "Next state: " << best_next_state << "\n";
    state = best_next_state;
    realize_state(predictions, time_interval);
}

Vehicle Vehicle::clone() {
    vector<double> config = {lanes_available, lane_width, 
        target_speed, max_acceleration, max_jerk};
    Vehicle clone = Vehicle(config);
    clone.state = state;

    clone.update_localization(x, y, s, d, yaw, v);

    //clone.a = a;
    //clone.v = v;
    //clone.s = s;
    //clone.d = d;
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
    //time_interval = config[5];
}

string Vehicle::display() {

	ostringstream oss;
	
    oss << "s:    " << this->s << "\n";
    oss << "d:    " << this->d << "\n";
    oss << "lane: " << this->get_lane() << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    oss << "target v: " << this->target_speed << "\n";
    oss << "max a:    " << this->max_acceleration << "\n";
    oss << "max jerk: " << this->max_jerk << "\n";
    
    return oss.str();
}

/*void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}*/

vector<double> Vehicle::state_at(float t) {

	/*
    Predicts state of vehicle in t seconds
    */
    //float s = this->s + this->v * t + this->a * t * t / 2;
    //float v = this->v + this->a * t;

    //std::cout << "coeffs_s: " << coeffs_s.size() << std::endl;

    double s_ = coeffs_s[0] + coeffs_s[1] * t + coeffs_s[2] * pow(t, 2) 
    + coeffs_s[3] * pow(t, 3) + coeffs_s[4] * pow(t, 4) 
    + coeffs_s[5] * pow(t, 5);

    //std::cout << "coeffs_d: "<< coeffs_d.size() << std::endl;

    double d_ = coeffs_d[0] + coeffs_d[1] * t + coeffs_d[2] * pow(t, 2) 
    + coeffs_d[3] * pow(t, 3) + coeffs_d[4] * pow(t, 4) 
    + coeffs_d[5] * pow(t, 5);

    return {s_, d_};
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

void Vehicle::realize_state(map<int, vector<vector<double>>> predictions, float time_interval) {
   
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
    	realize_keep_lane(predictions, time_interval);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
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

/*int Vehicle::_max_accel_for_lane(map<int, vector<vector<float>>> predictions, int lane, int s) {

	float delta_v_til_target = target_speed - v;
    float max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<float>>>::iterator it = predictions.begin();
    vector<vector<vector<float>>> in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<float>> v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	float min_s = 1000;
    	vector<vector<float>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	float next_pos = leading[1][1];
    	float my_next = s + this->v;
    	float separation_next = next_pos - my_next;
    	float available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}*/

void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions, float time_interval) {
    //this->a = _max_accel_for_lane(predictions, get_lane(), this->s);

    cout<<"Realizing keep lane..." << endl;
    map<int, vector<vector<double>>>::iterator it = predictions.begin();
    vector<vector<vector<double>>> in_front;

    double next_s;
    double next_v = target_speed;

    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<double>> v = it->second;
        
        if((get_lane(v[0][6]) == get_lane()) && (v[0][5] > s))
        {
        	in_front.push_back(v);
        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	double min_s = 1000;
    	vector<vector<double>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][5]-s) < min_s)
    		{
    			min_s = (in_front[i][0][5]-s);
    			leading = in_front[i];
    		}
    	}
    	
        double next_s = leading[1][5] - preferred_buffer;
        double next_v = (next_s - s) / time_interval;


        //next_pos = min(next_pos, waypoint_s);
        //float my_next = s + this->v;
    	//float separation_next = next_pos - my_next;
    	//float available_room = separation_next - preferred_buffer;
    	//max_acc = min(max_acc, available_room);
    } else {
        cout<< "Free lane!"<<endl;
    }

    if(next_v > target_speed) {
        cout<<"Above speed limit! " << next_v << endl;
        next_v = target_speed;
    }

    double next_a = (next_v - v) / time_interval;

    if(fabs(next_a) > max_acceleration) {
        cout<<"Above acc limit! " << next_a << endl;
        next_v = v + max_acceleration * time_interval * (next_a < 0 ? -1. : 1.);
        next_a = (next_v - v) / time_interval;
    }

    double jerk = (next_a - a) / time_interval;

    if(fabs(jerk) > max_jerk) {
        cout<<"Above jerk limit! " << jerk << endl;
        next_a = a + max_jerk * time_interval * (jerk < 0 ? -1. : 1.);
        next_v = v + next_a * time_interval;
    }

    next_s = s + next_v * time_interval + 0.5 * next_a * pow(time_interval, 2);
    //next_a = 0;

    cout<< "Trajectory from {s=" << s << ", v=" << v << ", a=" << a << "} to {s=" << next_s << ", v=" << next_v << ", a=" << next_a 
    << " in " << time_interval << " seconds" << endl;
    coeffs_s = JMT({s, v, a}, {next_s, next_v, next_a}, time_interval);

    //s = next_s;
    v = next_v;
    //a = next_a;
}

void Vehicle::realize_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    //this->a = _max_accel_for_lane(predictions, lane, s);
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
    for(float i = 0; i < horizon; i++)
    {
      //vector<float> check1 = state_at(i);
      //vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(state_at(i));
  	}
    return predictions;

}