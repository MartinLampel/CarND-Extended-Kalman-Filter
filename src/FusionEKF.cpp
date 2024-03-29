#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  
  Eigen::VectorXd x = VectorXd(4);
  Eigen::MatrixXd Q = MatrixXd(4, 4);
  
  // Initializing F
  Eigen::MatrixXd F = MatrixXd(4, 4);
  F << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Initializing P
  Eigen::MatrixXd P = MatrixXd(4, 4);
  P << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
      double rho = measurement_pack.raw_measurements_[0]; // range
      double phi = measurement_pack.raw_measurements_[1]; // bearing
      double rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
      
      float px = cos(phi) * rho;
      float py = sin(phi) * rho;
      float vx = cos(phi) * rho_dot;
      float vy = sin(phi) * rho_dot;

      // we are certain about the speed
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      ekf_.P_ <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;

      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_ ;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    return;
  }

  /**
   * Prediction
   */
 
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transition matrix update
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;


  double dt_2 = dt * dt; //dt^2
  double dt_3 = dt_2 * dt; //dt^3
  double dt_4 = dt_3 * dt; //dt^4
  double dt_4_4 = dt_4 / 4; //dt^4/4
  double dt_3_2 = dt_3 / 2; //dt^3/2
  
  ekf_.Q_ << dt_4_4 * noise_ax_, 0, dt_3_2 * noise_ax_, 0,
	         0, dt_4_4 * noise_ay_, 0, dt_3_2 * noise_ay_,
	         dt_3_2 * noise_ax_, 0, dt_2 * noise_ax_, 0,
 	         0, dt_3_2 * noise_ay_, 0, dt_2 * noise_ay_;

  ekf_.Predict();
  
  
  /**
   * Update
   */
  
  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

   if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
     ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
     ekf_.R_ = R_radar_;
     ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
     ekf_.H_ = H_laser_;
     ekf_.R_ = R_laser_;
     ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
