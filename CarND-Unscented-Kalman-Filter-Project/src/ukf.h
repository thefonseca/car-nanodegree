#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
    
    void AugmentedSigmaPoints(MatrixXd* Xsig_out);
    
    void SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t);
    
    void PredictMeanAndCovariance();
    
    void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out);
    
    void PredictLidarMeasurement(VectorXd* z_out, MatrixXd* Zsig_out, MatrixXd* S_out);
    
    void UpdateState(VectorXd z, VectorXd z_pred, MatrixXd Zsig, MatrixXd S);
    
    void UpdateNIS(VectorXd z_diff, MatrixXd S);
    
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Number of measurements processed
  int radar_measurement_count_;
  int lidar_measurement_count_;
  
  ///* Number of times NIS lower than upper threshold
  int radar_nis_upper_ok_count_;
  int lidar_nis_upper_ok_count_;
    
  ///* Number of times NIS above lower threshold
  int radar_nis_lower_ok_count_;
  int lidar_nis_lower_ok_count_;
  
  ///* NIS upper threshold
  float radar_nis_upper_threshold_;
  float lidar_nis_upper_threshold_;
  
  ///* NIS lower threshold
  float radar_nis_lower_threshold_;
  float lidar_nis_lower_threshold_;
  
  ///* NIS should be within threshold at least "percent_threshold_%" of the time
  float percent_threshold_;
    
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage measurement_pack);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage measurement_pack);
};

#endif /* UKF_H */
