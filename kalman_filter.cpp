#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */
  // Prediction formula: (I): x' = F*x +u   (II): P' = FPF^T +Q 
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();

  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  // update x_
  x_ = x_ + K* y;
  MatrixXd I = MatrixXd::Identity(4, 4); // 4x4 Identity only for 2D space
  P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Calculate h(x'): -- 3D Vector (rho, phi, rho_dot) more infos Lidar/Radar Lession 20.(!)
  VectorXd h(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  // calculate rho, phi and rho_dot
  float rho = sqrt(px*py+py*py);
  float phi = atan(py/px);  // In Radiant -- atan2 testen
  float rho_dot = (px*vx+py*vy)/rho;
  h << rho, phi, rho_dot;
  //quation for radar becomes y=z − h(x′):
  VectorXd y = z - h;
  // Update:
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + K* y;
  MatrixXd I = MatrixXd::Identity(4, 4); // 4x4 Identity only for 2D space
  P_ = (I-K*H_)*P_;




}
