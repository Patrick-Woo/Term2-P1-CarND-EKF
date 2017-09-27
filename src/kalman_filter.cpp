#include "kalman_filter.h"
#define pi 3.14159265
//#include <math.h>  //#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;    // object state
  P_ = P_in;    // object uncertaintity covariance matrix
  F_ = F_in;    // state transition matrix
  H_ = H_in;    // measurement fuction matrix
  R_ = R_in;    // measurement(sensor) covariance matrix
  Q_ = Q_in;    // process(prediction) covariance matrix
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_ ; // There is no external motion Bu and we do not have to add "+v" as "v"=0
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

//PM... this function is for lidar measurement update
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	// Kalman filter update step. Equations from the lectures
	  VectorXd y = z - H_ * x_; // error calculation  //PM...这里的z是真值(px,py),所以H_ * x_也该是2行1列的.
	  KF(y);
}

//PM... this function is for radar measurement update
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // PM...Recalculate x object state to rho, theta, rho_dot coordinates
  //formula
  //h(x​′​​)=​⎝​⎛​​​ρ​ϕ​​ρ​˙​​​​​⎠​⎞​​=​⎝​⎜​⎜​⎛​​​√​p​′​​​x​2​​+p​′​​​y​2​​​​​​,  arctan(p​y​′​​/p​x​′​​)​, ​√​p​′​​​x​2​​+p​′​​​y​2​​​​​​​p​x​′​​v​x​′​​+p​y​′​​v​y​′​​​​​​​⎠​⎟​⎟​⎞​​
      double rho = sqrt(double(x_(0)*x_(0) + x_(1)*x_(1)));    //PM... add double explicit transfer
	  //double theta = atan(double(x_(1) / x_(0)));              //PM... add double explicit transfer
      double theta = atan2(double(x_(1)) , double(x_(0)));       //PM... angel normalization -pi to pi
      double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;

	  VectorXd h = VectorXd(3); // h(x_) or h(x​′​​), see the formula before
	  h << rho, theta, rho_dot;

	  VectorXd y = z - h;    //PM...这里的z是真值(rho,phi,rho-dot),所以h(x')也该是3行1列的.

      //PM...angle normalization
	  //if (fabs(double(y[1])) < 0.1)
	  //		  {y[1]=0.1;}
	  if (double(y[1])>pi)
	  {
		  y[1]=y[1]-2*pi;
	  }
	  else if (double(y[1])<-pi)
	  {
		  y[1]=y[1]+2*pi;
	  }

	  // Calculations are essentially the same to the Update function
	  KF(y);
}

// Universal update Kalman Filter step. Equations from the lectures
void KalmanFilter::KF(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
