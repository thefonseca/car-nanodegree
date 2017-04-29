#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    VectorXd y = z - H_ * x_;
    MatrixXd S  = H_ * P_ * H_.transpose() + R_;
    MatrixXd K =  P_ * H_.transpose() * S.inverse();
    
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    
    /**
     * Update the state by using Extended Kalman Filter equations.
     */
    
    VectorXd y =  z - Tools::to_polar(x_);
    
    // keep bearing angle (phi) between -PI and PI
    while(y(1) > M_PI) y(1) -= 2 * M_PI;
    while(y(1) < -M_PI) y(1) += 2 * M_PI;
    
    MatrixXd H_jacobian = Tools::CalculateJacobian(x_);
    
    // Ignore radar update if Jacobian is not defined
    if (!(H_jacobian.array() != 0.0).any()) {
        return;
    }
    
    MatrixXd S  = H_jacobian * P_ * H_jacobian.transpose() + R_;
    MatrixXd K =  P_ * H_jacobian.transpose() * S.inverse();
    
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_jacobian) * P_;
}
