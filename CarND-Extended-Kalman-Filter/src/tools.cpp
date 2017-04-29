#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        std::cout<< "Error: number of estimations must match number of ground truth values";
        return rmse;
    }
    
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        rmse = rmse.array() + (estimations[i] - ground_truth[i]).array().pow(2);
    }
    
    //calculate the mean
    rmse /= estimations.size();
    
    //calculate the squared root
    rmse = rmse.array().sqrt();
    
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    
    MatrixXd Hj = MatrixXd::Zero(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //check division by zero
    float pxy = pow(px, 2) + pow(py, 2);
    float r_pxy = sqrt(pxy);
    
    if (fabs(pxy) < 0.0001) {
        std::cout<<"Error in CalculateJacobian(): division by zero"<<std::endl;
        return Hj;
    }
    
    //compute the Jacobian matrix
    
    Hj << px/r_pxy, py/r_pxy, 0, 0,
        -py/pxy, px/pxy, 0, 0,
        py*(vx*py - vy*px)/pow(pxy, 1.5), px*(vy*px - vx*py)/pow(pxy, 1.5), px/r_pxy, py/r_pxy;
    
    return Hj;
}

VectorXd Tools::to_polar(const VectorXd &x) {
    
    VectorXd p = Eigen::VectorXd(3);
    
    float px2_py2 = sqrt(x(0) * x(0) + x(1) * x(1));
    p << px2_py2,
         atan2(x(1), x(0)),
         (x(0) * x(2) + x(1) * x(3)) / px2_py2;
    
    return p;
}
