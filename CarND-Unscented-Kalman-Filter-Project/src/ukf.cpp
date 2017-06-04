#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = .3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .3;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  time_us_ = 0;
  weights_ = VectorXd(2 * n_aug_ + 1);
    
  //set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));
  }
    
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  // Normalized Innovation Squared (NIS) variables
  radar_measurement_count_ = 0;
  radar_nis_upper_ok_count_ = 0;
  radar_nis_lower_ok_count_ = 0;
  radar_nis_upper_threshold_ = 7.815; // chi2 0.050 (3 degrees of freedom)
  radar_nis_lower_threshold_ = 0.352; // chi2 0.950 (3 degrees of freedom)
    
  lidar_measurement_count_ = 0;
  lidar_nis_upper_ok_count_ = 0;
  lidar_nis_lower_ok_count_ = 0;
  lidar_nis_upper_threshold_ = 5.991; // chi2 0.050 (2 degrees of freedom)
  lidar_nis_lower_threshold_ = 0.103; // chi2 0.950 (2 degrees of freedom)
    
  percent_threshold_ = 0.95;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    
    /****************************************************************************
     *  Initialization
    ****************************************************************************/
    if (!is_initialized_) {
        
        /**
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        
        // first measurement
        //cout << "EKF: " << endl;
        x_ << 1, 1, 1, 1, 1;
        
        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 1;
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            
            float ro = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            x_(0) = ro * cos(phi);
            x_(1) = ro * sin(phi);
            
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            x_(0) = measurement_pack.raw_measurements_(0);
            x_(1) = measurement_pack.raw_measurements_(1);
        }
        
        time_us_ = measurement_pack.timestamp_;
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /****************************************************************************
     *  Prediction
    ****************************************************************************/
    
    /**
     * Update the state transition matrix F according to the new elapsed time.
     - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    //compute the time elapsed between the current and previous measurements
    //dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
    time_us_ = measurement_pack.timestamp_;
    
    Prediction(dt);
    
    /****************************************************************************
     *  Update
     ***************************************************************************/
    
    /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        UpdateRadar(measurement_pack);
        
    } else {
        // Laser updates
        UpdateLidar(measurement_pack);
    }
    
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
    
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    //create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;
    x_aug(n_x_ + 1) = 0;
    
    //create augmented covariance matrix
    MatrixXd Q = MatrixXd(2, 2);
    Q << std_a_ * std_a_, 0,
    0, std_yawdd_ * std_yawdd_;
    
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug.bottomRightCorner(n_aug_ - n_x_, n_aug_ - n_x_) = Q;
    
    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    
    float c = sqrt(lambda_ + n_aug_);
    MatrixXd m = A * c;
    
    for (int i = 1; i <= n_aug_; i++) {
        Xsig_aug.col(i) = x_aug + m.col(i-1);
        Xsig_aug.col(i + n_aug_) = x_aug - m.col(i-1);
    }
    
    *Xsig_out = Xsig_aug;
}


void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t) {
    
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        
        VectorXd col = Xsig_aug.col(c);
        VectorXd x = col.head(n_x_);
        VectorXd noise = col.tail(n_aug_ - n_x_);
        
        if (fabs(x(4)) > 0.001) {
            x(0) += x(2)/x(4) * (sin(x(3) + x(4) * delta_t) - sin(x(3)));
            x(1) += x(2)/x(4) * (cos(x(3)) - cos(x(3) + x(4) * delta_t));
            
        } else {
            x(0) += x(2) * cos(x(3)) * delta_t;
            x(1) += x(2) * sin(x(3)) * delta_t;
        }
        
        x(3) += x(4) * delta_t;
        
        x(0) += 0.5 * delta_t * delta_t * cos(col(3)) * noise(0);
        x(1) += 0.5 * delta_t * delta_t * sin(col(3)) * noise(0);
        x(2) += delta_t * noise(0);
        x(3) += 0.5 * delta_t * delta_t * noise(1);
        x(4) += delta_t * noise(1);
        
        Xsig_pred_.col(c) = x;
    }
    
}

void UKF::PredictMeanAndCovariance() {
    
    //predict state mean
    x_.fill(0.0);
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        x_ = x_ + weights_(c) * Xsig_pred_.col(c);
    }
    
    //predict state covariance matrix
    P_.fill(0.0);
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(c) - x_;
        
        //angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
        
        P_ = P_ + weights_(c) * x_diff * x_diff.transpose();
        
    }
    
}


void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out) {
    
    //create matrix for sigma points in measurement space
    int n_z = z_out->size();
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    
    //transform sigma points into measurement space
    for (int c=0; c< 2 * n_aug_ + 1; c++) {
        VectorXd z = VectorXd(n_z);
        VectorXd x = Xsig_pred_.col(c);
        
        z(0) = sqrt(x(0) * x(0) + x(1) * x(1));
        z(1) = atan2(x(1), x(0));
        z(2) = (x(0) * cos(x(3)) * x(2) + x(1) * sin(x(3)) * x(2)) / z(0);
        Zsig.col(c) = z;
    }
    
    z_pred.fill(0.0);
    //calculate mean predicted measurement
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        z_pred += weights_(c) * Zsig.col(c);
    }
    
    S.fill(0.0);
    //calculate measurement covariance matrix S
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        VectorXd z_diff = Zsig.col(c) - z_pred;
        
        //angle normalization
        while (z_diff(1) > M_PI)  z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
        
        S = S + weights_(c) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_radr_ * std_radr_, 0, 0,
         0, std_radphi_ * std_radphi_, 0,
         0, 0, std_radrd_ * std_radrd_;
    S = S + R;
    
    //write result
    *z_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
}


void UKF::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out) {
    
    //create matrix for sigma points in measurement space
    int n_z = z_out->size();
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    
    //transform sigma points into measurement space
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        // extract values for better readibility
        double p_x = Xsig_pred_(0,c);
        double p_y = Xsig_pred_(1,c);
        
        // measurement model
        Zsig(0,c) = p_x;
        Zsig(1,c) = p_y;
    }

    z_pred.fill(0.0);
    //calculate mean predicted measurement
    for (int c=0; c < 2 * n_aug_ + 1; c++) {
        z_pred += weights_(c) * Zsig.col(c);
    }
    
    //measurement covariance matrix S
    S.fill(0.0);
    for (int c = 0; c < 2 * n_aug_ + 1; c++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(c) - z_pred;
        S = S + weights_(c) * z_diff * z_diff.transpose();
    }
    
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R << std_laspx_ * std_laspx_, 0,
         0, std_laspy_ * std_laspy_;
    
    S = S + R;
    
    //write result
    *z_out = z_pred;
    *S_out = S;
    *Zsig_out = Zsig;
}


void UKF::UpdateState(VectorXd z, VectorXd z_pred, MatrixXd Zsig, MatrixXd S) {
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, z_pred.size());
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        
        if (z.size() == 3) {
            while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
            while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
        }
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    if (z.size() == 3) {
        while (z_diff(1) >  M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    }
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    
    UpdateNIS(z_diff, S);
}

void UKF::UpdateNIS(VectorXd z_diff, MatrixXd S) {
    
    // Normalized Innovation Squared (NIS)
    float nis = z_diff.transpose() * S.inverse() * z_diff;
    string sensor;
    float percent_upper_ok, percent_lower_ok;
    
    if (z_diff.size() == 3) { // radar
        
        sensor = "Radar";
        
        radar_measurement_count_++;
        
        if (nis > radar_nis_lower_threshold_) {
            radar_nis_lower_ok_count_++;
        }
        
        if (nis < radar_nis_upper_threshold_) {
            radar_nis_upper_ok_count_++;
        }
        
        percent_upper_ok = (radar_nis_upper_ok_count_ * 1./radar_measurement_count_);
        percent_lower_ok = (radar_nis_lower_ok_count_ * 1./radar_measurement_count_);
        
        
    } else { // Lidar
        
        sensor = "Lidar";
        
        lidar_measurement_count_++;
        
        if (nis > lidar_nis_lower_threshold_) {
            lidar_nis_lower_ok_count_++;
        }
        
        if (nis < lidar_nis_upper_threshold_) {
            lidar_nis_upper_ok_count_++;
        }
        
        percent_upper_ok = (lidar_nis_upper_ok_count_ * 1./lidar_measurement_count_);
        percent_lower_ok = (lidar_nis_lower_ok_count_ * 1./lidar_measurement_count_);
        
    }
    
    std::cout<< sensor << " NIS within upper threshold: " << percent_upper_ok * 100 << "%";
    
    if (percent_upper_ok > percent_threshold_) {
        std::cout<< " - OK!\n";
        
    } else {
        std::cout<< " - NOT OK!\n";
    }
    
    std::cout<< sensor << " NIS within lower threshold: " << percent_lower_ok * 100 << "%";
    
    if (percent_lower_ok > percent_threshold_) {
        std::cout<< " - OK!\n";
        
    } else {
        std::cout<< " - NOT OK!\n";
    }
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     TODO:
     
     Complete this function! Estimate the object's location. Modify the state
     vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */
    
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    AugmentedSigmaPoints(&Xsig_aug);
    SigmaPointPrediction(Xsig_aug, delta_t);
    PredictMeanAndCovariance();
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
    
    int n_z = 3;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    MatrixXd S = MatrixXd(n_z, n_z);
    VectorXd z = measurement_pack.raw_measurements_;
    
    PredictRadarMeasurement(&z_pred, &Zsig, &S);
    UpdateState(z, z_pred, Zsig, S);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
    /**
     TODO:
     
     Complete this function! Use lidar data to update the belief about the object's
     position. Modify the state vector, x_, and covariance, P_.
     
     You'll also need to calculate the lidar NIS.
     */
    
    int n_z = 2;
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    MatrixXd S = MatrixXd(n_z, n_z);
    VectorXd z = measurement_pack.raw_measurements_;
    
    PredictLidarMeasurement(&z_pred, &Zsig, &S);
    UpdateState(z, z_pred, Zsig, S);
}
