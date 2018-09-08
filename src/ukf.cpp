#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .3;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  n_radar = 3;  // Radar measurement dimension   
  n_lidar = 2;  // Lidar measurement dimension
  n_x_ = 5;     // Set state dimension
  n_aug_ = 7;   // Set augmented state dimension
  n_sigma = 2 * n_aug_ + 1;   // Set number of sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  is_initialized_ = false;
  lambda_ = 3 - n_aug_;
  NIS_lidar = 0;
  NIS_radar = 0;

  // Initialize weights 
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // Initialize covariance matrix;
  P_ << 0.15, 0, 0, 0, 0,
        0, 0.15, 0, 0, 0,
        0,    0, 1, 0, 0,
        0,    0, 0, 1, 0,
        0,    0, 0, 0, 1;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  if (!is_initialized_) {
    // init timestamp
    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_(0);
      double yaw = meas_package.raw_measurements_(1);
      // double rho_dot = meas_package.raw_measurements_(2);

      double px = rho * cos(yaw);
      double py = rho * sin(yaw);
      /**
       * a radar measurement does not contain enough information 
       * to determine the state variable velocities v_x  v_y
       */
      x_ << px, py, 0, 0, 0; 
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float px = meas_package.raw_measurements_(0);
      float py = meas_package.raw_measurements_(1);
      x_ << px, py, 0, 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  const double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  
  time_us_ = meas_package.timestamp_;
  Prediction(dt);
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
    * Use the sensor type to perform the update step.
    * Update the state and covariance matrices.
  */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
    // cout << "NIS Score: " << NIS_radar << '\n';
  } else {
    // Lidar updates
    UpdateLidar(meas_package);
    // cout << "NIS Score: " << NIS_lidar << '\n';
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  /*****************************************************************************
   *  Generate Sigma points
   ******************************************************************************/
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma);
  
  // Setup augmented state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  // Setup augmented covariance matrix
  P_aug.setZero();
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  
  // Create sqrt matrix 
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i=0; i<n_aug_; ++i) {
      Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
      Xsig_aug.col(i + n_aug_ + 1) = x_aug - sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i); 
  }

  /*****************************************************************************
   *  Make Predictions with Sigma points
   ******************************************************************************/
  for (int i = 0; i < n_sigma; ++i)
    {
        double px = Xsig_aug(0, i);
        double py = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double noise_a = Xsig_aug(5, i);
        double noise_yawdd = Xsig_aug(6, i);

        double px_p, py_p;
        //avoid division by zero
        if (yawd > 0.001) {
            px_p = px + v/yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
            py_p = py + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
        }
        else {
            px_p = px + v * delta_t * cos(yaw);
            py_p = py + v * delta_t * sin(yaw);
        }
        double v_p = v;
        double phi_p = yaw + yawd * delta_t;
        double phi_dot_p = yawd;

        //add noise
        px_p = px_p + 0.5 * noise_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * noise_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + noise_a * delta_t;

        phi_p = phi_p + 0.5 * noise_yawdd * delta_t * delta_t;
        phi_dot_p = phi_dot_p + noise_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = phi_p;
        Xsig_pred_(4,i) = phi_dot_p;
    }
  /*****************************************************************************
   *  Prediction Mean and Covariance from Sigma Points
   ******************************************************************************/
  // Predict state mean
  x_.fill(0);
  for (int i=0; i<n_sigma; ++i)
      x_ += weights_(i) * Xsig_pred_.col(i);

  // Predict state covariance matrix
  VectorXd residual;
  P_.fill(0);
  for (int i=0; i<n_sigma; ++i) {
      residual = Xsig_pred_.col(i) - x_;
      residual(3) = atan2(sin(residual(3)), cos(residual(3))); // Angle Normalization
      P_ += weights_(i) * residual * residual.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // NOTE: since H is lidar for lidar measurement, regular kalman filter can be used,
  //       although the result of using UKF should be same as KF.
  VectorXd residual;
  MatrixXd R = MatrixXd(n_lidar, n_lidar);
  MatrixXd S = MatrixXd(n_lidar, n_lidar);
  MatrixXd H = MatrixXd(n_lidar, n_x_);
  MatrixXd Zsig = MatrixXd(n_lidar, n_sigma);
  MatrixXd z_pred = VectorXd(n_lidar);

  /*****************************************************************************
   *  Measurement Prediction
   ******************************************************************************/
  // Setup matrix H to map from state space to measurement space
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;
  
  // Create measurement sigma points
  Zsig = H * Xsig_pred_;

  // Predict measurement at t+1
  z_pred.setZero();
  for (int i=0; i<n_sigma; ++i)
    z_pred += weights_(i) * Zsig.col(i);

  //calculate innovation covariance matrix S  
  R.setZero();
  S.setZero();
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;
  for (int i=0; i<n_sigma; ++i) {
      residual = Zsig.col(i) - z_pred;
      S += weights_(i) * residual * residual.transpose();
  }
  S += R;

  /*****************************************************************************
   *  Measurement Update
   ******************************************************************************/
  VectorXd z = meas_package.raw_measurements_;
  VectorXd x_residual, z_residual;
  MatrixXd Tc = MatrixXd(n_x_, n_lidar);

  // Calculate Cross-Correlation
  Tc.setZero();
  for (int i=0; i<n_sigma; ++i) {
    x_residual = Xsig_pred_.col(i) - x_;
    z_residual = Zsig.col(i) - z_pred;
    Tc += weights_(i) * x_residual * z_residual.transpose();
  }

  // Calculte Kalman Gain
  MatrixXd K = Tc * S.inverse();

  // State Update
  z_residual = z - z_pred;
  x_ += K * z_residual;

  // Covariance Update
  P_ -= K * S * K.transpose();

  // Calculate NIS score
  NIS_lidar = z_residual.transpose() * S.inverse() * z_residual;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  VectorXd residual;
  MatrixXd R = MatrixXd(n_radar, n_radar);
  MatrixXd S = MatrixXd(n_radar, n_radar);
  MatrixXd H = MatrixXd(n_radar, n_x_);
  MatrixXd Zsig = MatrixXd(n_radar, n_sigma);
  MatrixXd z_pred = VectorXd(n_radar);

  /*****************************************************************************
   *  Measurement Prediction
   ******************************************************************************/
  double px, py, vx, vy, v, yaw;
    
  //transform sigma points into measurement space
  for (int i=0; i<n_sigma; ++i) {
      px = Xsig_pred_(0, i);
      py = Xsig_pred_(1, i);
      v = Xsig_pred_(2, i);
      yaw = Xsig_pred_(3, i);
      
      vx = cos(yaw) * v;
      vy = sin(yaw) * v;

      Zsig(0, i) = sqrt(px * px + py * py);   // rho
      Zsig(1, i) = atan2(py, px);             // phi
      if (Zsig(0, i) < 0.00001) {
          std::cout << "WARNING - DIVIDE BY ZERO: rho is close to zero\n";
          Zsig(0, i) = 0.00001;
      }
      Zsig(2, i) = (px * vx + py * vy) / Zsig(0,i); // rho_dot
  }
  
  //calculate mean predicted measurement
  z_pred.setZero();
  for (int i=0; i<n_sigma; ++i)
      z_pred += weights_(i) * Zsig.col(i);

  //calculate innovation covariance matrix S
  R.setZero();
  S.setZero();
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  for (int i=0; i<n_sigma; ++i) {
      residual = Zsig.col(i) - z_pred;
      residual(1) = atan2(sin(residual(1)), cos(residual(1)));
      S += weights_(i) * residual * residual.transpose();
  }
  S += R;



  /*****************************************************************************
   *  Measurement Update
   ******************************************************************************/
  VectorXd z = meas_package.raw_measurements_;
  VectorXd x_residual, z_residual;
  MatrixXd Tc = MatrixXd(n_x_, n_radar);

  // Calculate Cross-Correlation
  Tc.setZero();
  for (int i=0; i<n_sigma; ++i) {
      x_residual = Xsig_pred_.col(i) - x_;
      z_residual = Zsig.col(i) - z_pred;
      // Normalize yaw
      x_residual(3) = atan2(sin(x_residual(3)), cos(x_residual(3)));
      z_residual(1) = atan2(sin(z_residual(1)), cos(z_residual(1)));

      Tc += weights_(i) * x_residual * z_residual.transpose();
  }

  // Calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // State Update 
  z_residual = z - z_pred;
  z_residual(1) = atan2(sin(z_residual(1)), cos(z_residual(1))); // Normalize yaw
  x_ += K * z_residual;

  // Covariance Update
  P_ -= K * S * K.transpose();

  // Calculate NIS score
  NIS_radar = z_residual.transpose() * S.inverse() * z_residual;
}
