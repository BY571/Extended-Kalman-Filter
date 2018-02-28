#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  // Radar and Lidar - Lesson 22.
  VectorXd rmse(4);
	rmse << 0,0,0,0;
  // check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}
  //accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}
  //calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
  // Checking Size of x_state
  if (x_state.size() != 4){
    std::cout << "ERROR: By computing Jacobi-Matrix. x_state is not 2D!"<< endl;
    return Hj;
  }
  // Position and velocity 
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  // Precalculations
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = c2*c1;
  // Testing for dividing by Zero
  if (c1 == 0)
  {
    std::cout<<"ERROR: Dividing by Zero! (JM c1)"<<endl;
    return Hj;
  }
  // Final Jacobi-Matrix script (75)
  Hj<< px/c2, py/c2, 0, 0,
        -py/c1, px/c1, 0, 0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;
  
  return Hj;


}
