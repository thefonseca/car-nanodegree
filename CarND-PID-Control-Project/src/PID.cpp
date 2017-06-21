#include "PID.h"
#include <iostream>
#include <cmath>

/*
* TODO: Complete the PID class.
*/

PID::PID() {
}

PID::~PID() {}

void PID::Twiddle() {
    
    double sum_dp = dp[0] + dp[1] + dp[2];
    
    if (sum_dp < tol) {
        return;
    }
    
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
    
    if (decremented) {
        
        if (best_error < 0 || total_error < best_error) {
            best_error = total_error;
            dp[param_index] *= 1.1;
            
        } else {
            p[param_index] += dp[param_index];
            dp[param_index] *= 0.9;
        }
        
        decremented = false;
        incremented = false;
        
        param_index = (param_index + 1) % 3;
        
    } else if (incremented) {
        
        if (best_error < 0 || total_error < best_error) {
            best_error = total_error;
            dp[param_index] *= 1.1;
        }
        
        p[param_index] -= 2 * dp[param_index];
        
        decremented = true;
        incremented = false;
        
    } else {
        
        p[param_index] += dp[param_index];
        
        decremented = false;
        incremented = true;
    }
    
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    total_error = 0;
}

void PID::Init(double Kp, double Ki, double Kd) {
    
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0.;
    i_error = 0.;
    d_error = 0.;
    
    tol = 0.01;
    
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
    
    for (int i = 0; i < 3; i++) {
        dp[i] = .1 * p[i];
    }
    
    incremented = false;
    decremented = false;
    total_error = 0;
    best_error = -1;
    steps_stabilization = 100;
    twiddle_period = 500;
    step = 0;
    param_index = 0;
}

void PID::UpdateError(double cte) {
    
    if (fabs(cte) < fabs(p_error)) { // prevent i_error from exploding
        i_error += cte;
    }
    
    d_error = cte - p_error;
    p_error = cte;
    
    if (step > steps_stabilization) {
        total_error += (cte * cte);
        
        if (step % twiddle_period == 0) {
            std::cout<< "Twiddle: " << step << "\n";
            Twiddle();
        }
    }
    
    //std::cout<< "Step: " << step << "\n";
    //std::cout<< p_error << " : " << i_error << " : " << d_error << "\n";
    //std::cout<< Kp << " : " << Ki << " : " << Kd << "\n";
    
    step++;
}

double PID::TotalError() {
    
    double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
    
    //std::cout<< t_error << " : " << Kp <<  "\n";
    return total_error;
}

