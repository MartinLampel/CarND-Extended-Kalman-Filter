#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  
  I_ = MatrixXd::Identity(F_.rows(), F_.rows());
}

void KalmanFilter::Predict() {
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_ * x_;
  UpdateStep(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  VectorXd h = VectorXd(3);
  
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px + py*py);
  
  if (rho < .00001) {
    px += .001;
    py += .001;
    rho = sqrt(px*px + py*py);
  }
     
  double theta = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;
 
  
  h << rho, theta, rho_dot;
  
  VectorXd y = z - h;
 
  for (; y(1) < -M_PI; y(1) += 2*M_PI) {}
  for (; y(1) > M_PI;  y(1) -= 2*M_PI) {}
  
  UpdateStep(y);    
}

 /**
   * Updates the state by using standard Kalman Filter equations
   * Should be called by the Update or UpdateEKF after computing y
   * @param y The difference between the measurement and the predicted output: y = z - H * x
   */
void KalmanFilter::UpdateStep(const Eigen::VectorXd &y) {
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  
  //new estimate
  x_ = x_ + (K * y);  
  P_ = (I_ - K * H_) * P_;
}
