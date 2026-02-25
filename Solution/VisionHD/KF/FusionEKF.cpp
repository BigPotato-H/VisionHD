#include "FusionEKF.h"
//#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3, 4);

	// Unroll state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Pre-compute some term which recur in the Jacobian
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = c1 * c2;

	// Sanity check to avoid division by zero
	if (std::abs(c1) < 0.0001) {
		std::cout << "Error in CalculateJacobian. Division by zero." << std::endl;
		return Hj;
	}

	// Actually compute Jacobian matrix
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py* (vx * py - vy * px) / c3, px* (vy * px - vx * py) / c3, px / c2, py / c2;

	return Hj;

}


/* Constructor. */
FusionEKF::FusionEKF(const vector<double>& pose) {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initialize measurement covariance matrix - M_ini
  R_ini_ = MatrixXd::Identity(6, 6);
  R_map_ = MatrixXd::Identity(6, 6);


  R_map_ << 0.1, 0, 0, 0, 0, 0,
	  0, 0.1, 0, 0, 0, 0,
	  0, 0, 0.1, 0, 0, 0,
	  0, 0, 0, 0.5, 0, 0,
	  0, 0, 0, 0, 0.5, 0,
	  0, 0, 0, 0, 0, 0.5;
  //measurement covariance matrix - M_MAP
  //R是观测噪声
  R_ini_ << 0.0225, 0, 0, 0, 0, 0,
	  0, 0.0225, 0, 0, 0, 0,
	  0, 0, 0.0225, 0, 0, 0,
	  0, 0, 0, 0.001, 0, 0,
	  0, 0, 0, 0, 0.001, 0,
	  0, 0, 0, 0, 0, 0.001;

  // ini - measurement matrix
  H_ini_ = MatrixXd::Identity(6, 6);
  //H_ini_	<<	1, 0, 0, 0,
		//		0, 1, 0, 0;
  H_map_ = MatrixXd::Identity(6, 6);

  // map - jacobian matrix
  Hj_ = MatrixXd::Identity(6, 6);
  //Hj_		<<	0, 0, 0, 0,
		//		0, 0, 0, 0,
		//		0, 0, 0, 0;

  // Initialize state covariance matrix P
  ekf_.P_ = MatrixXd::Identity(6, 6);
  //ekf_.P_	<<	1,	0,	0,	 0,
		//		0,	1,	0,	 0,
		//		0,	0, 1000, 0,
		//		0,	0, 0,	1000;

  // Initial transition matrix F_
  ekf_.F_ = MatrixXd::Identity(6, 6);
  //ekf_.F_ <<	1, 0, 1, 0,
		//		0, 1, 0, 1,
		//		0, 0, 1, 0,
		//		0, 0, 0, 1;

  //过程噪声 Initialize process noise covariance matrix
  ekf_.Q_ = MatrixXd::Identity(6, 6) * 0.01;
  //ekf_.Q_ <<	0, 0, 0, 0,
		//		0, 0, 0, 0,
		//		0, 0, 0, 0,
		//		0, 0, 0, 0;

  // Initialize ekf state
  ekf_.x_ = VectorXd::Zero(6);
  //ekf_.x_ << 1, 1, 1, 1;


  /*****************************************************************************
 *  Initialization
 ****************************************************************************/
  if (!is_initialized_)
  {
	  // Initialize state
	  ekf_.x_ = VectorXd::Zero(6);
	  for (int i = 0; i < 6; i++)
	  {
		  ekf_.x_(i) = pose[i];
	  }
	  is_initialized_ = true;
  }
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

float calQ(float reg_prop)
{
	float scale = 1.0;
	if (reg_prop > 0.4)
	{
		scale = 0.1;
	}
	else if (reg_prop < 0.1)
	{
		scale = 10;
	}
	else
	{
		scale = 1;
	}

	return scale;
}

VectorXd FusionEKF::ProcessMeasurement(const vector<double>& init_pos, const CamPose&cp, bool is_in_intersection) {
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
	ekf_.Q_ = ekf_.Q_ * calQ(cp.regist_probability);

	ekf_.Predict();

	double scale = 1.0;
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  ekf_.R_ = R_map_ * scale;
  ekf_.H_ = H_map_;

  //配置值
  VectorXd z_init = VectorXd::Zero(6);
  for (int i = 0; i < 6; i++)
  {
	  z_init(i) = init_pos[i];
  }
  ekf_.Update(z_init);

  //测量值
  VectorXd z_map = VectorXd::Zero(6);
  for (int i = 0; i < 6; i++)
  {
	  z_map(i) = cp.camPose[i];
  }

  ekf_.Update(z_map);

  return ekf_.x_;
}
