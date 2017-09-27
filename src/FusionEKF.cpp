#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 0.0001 // A very small number
#define EPS2 0.0000001

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices.
  // As the laser and radar R&H matrices are quite different, we define their shapes individually.

  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  //patrick comments: R matrix is the sensor noise which can be provided by the sensor manufacturer.
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //PM... to initialize the H_laser_ matrix
  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;

  //PM...Hj need to be calculated by px,py,vx,vy. So, we just initialize its shape, not its value.
  /*pre-compute a set of terms to avoid repeated calculation
  	float c1 = px*px+py*py;
  	float c2 = sqrt(c1);
  	float c3 = (c1*c2);

  	//check division by zero
  	if(fabs(c1) < 0.0001){
  		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  		return Hj;
  	}

  	//compute the Jacobian matrix
  	Hj << (px/c2), (py/c2), 0, 0,
  		  -(py/c1), (px/c1), 0, 0,
  		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
   */
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;     //PM...

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	float rho = measurement_pack.raw_measurements_[0]; // range
    	float phi = measurement_pack.raw_measurements_[1]; // bearing
    	float rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
    	// Coordinates convertion from polar to cartesian
    	float x = rho * cos(phi);
    	float y = rho * sin(phi);
    	//Although radar gives velocity data in the form of the range rate ​ρ​˙​​,
    	//a radar measurement does not contain enough information to
    	//determine the state variable velocities v​x​​ and v​y​​.
    	//You can, however, use the radar measurements ρ and ϕ to
    	//initialize the state variable locations p​x​​ and p​y
    	//PM... and use the following function to initialize the approximate vx and vy​​
    	float vx = rho_dot * cos(phi);
    	float vy = rho_dot * sin(phi);
    	ekf_.x_ << x, y, vx , vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
     // PM...We don't know velocities from the first measurement of the LIDAR, so, we use zeros
    	ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    //PM...init the matrix P and deal with the special case of init problems
    // Deal with the special case initialisation problems
        if (fabs(float(ekf_.x_(0))) < EPS and fabs(float(ekf_.x_(1))) < EPS){     //PM...add float transfer
    		ekf_.x_(0) = EPS;
    		ekf_.x_(1) = EPS;
    	}
    	// Initial covariance matrix
        ekf_.P_ = MatrixXd(4, 4);
        ekf_.P_ << 1, 0, 0, 0,
    			   0, 1, 0, 0,
    			   0, 0, 1000, 0,
    			   0, 0, 0, 1000;
        // Print the initialization results
        cout << "EKF init: " << ekf_.x_ << endl;
        // Save the initiall timestamp for dt calculation
        previous_timestamp_ = measurement_pack.timestamp_;


    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //PM...update the timestamp and use the delta t to
  //calculate the state transition F and measurement noise Q

  // Calculate the timestamp between measurements in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_);
    dt /= 1000000.0; // convert micros to s
    previous_timestamp_ = measurement_pack.timestamp_;

    // State transition matrix update
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, dt, 0,
  			0, 1, 0, dt,
  			0, 0, 1, 0,
  			0, 0, 0, 1;
    // Noise covariance matrix Q computation
    // Noise values from the task
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    // Precompute some usefull values to speed up calculations of Q
    float dt_2 = dt * dt; //dt^2
    float dt_3 = dt_2 * dt; //dt^3
    float dt_4 = dt_3 * dt; //dt^4
    float dt_4_4 = dt_4 / 4; //dt^4/4
    float dt_3_2 = dt_3 / 2; //dt^3/2
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4_4 * noise_ax, 0, dt_3_2 * noise_ax, 0,
  	         0, dt_4_4 * noise_ay, 0, dt_3_2 * noise_ay,
  	         dt_3_2 * noise_ax, 0, dt_2 * noise_ax, 0,
   	         0, dt_3_2 * noise_ay, 0, dt_2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);   //PM... Use Jacobian instead of H
	  	ekf_.R_ = R_radar_;
	  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);   //PM...UpdateEKF is radar update function
  } else {
    // Laser updates
	    ekf_.H_ = H_laser_;
	  	ekf_.R_ = R_laser_;
	  	ekf_.Update(measurement_pack.raw_measurements_);    //PM...Update is lidar update function
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
