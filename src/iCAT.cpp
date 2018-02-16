#include "iCAT.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

iCAT::iCAT(int DoF):TPIK(DoF){
	I_=Eigen::MatrixXd::Identity(DoF_,DoF_);
	Q_=I_;
};
iCAT::~iCAT(){};
void iCAT::ComputeYStep(Eigen::MatrixXd J,Eigen::MatrixXd Alpha,Eigen::VectorXd xdot){
	Eigen::MatrixXd barG = J * Q_;
	Eigen::MatrixXd barGtraspAA = barG.transpose() * Alpha * Alpha;
	// same regularization matrices, see above
	Eigen::MatrixXd T = (I_ - Q_).transpose() * (I_ - Q_);
	Eigen::MatrixXd H = barG.transpose() * (Eigen::MatrixXd::Identity(J.rows(),J.rows()) - Alpha) * Alpha * barG;
	Eigen::MatrixXd W;
	Eigen::MatrixXd barGpinv;
	//Eigen::MatrixXd W = barG
	//				* (barGtraspAA * barG + T + H).RegPseudoInverse(params.threshold, params.lambda, SVDvalues[SVDPARAM_W].mu,
	//						SVDvalues[SVDPARAM_W].flag) * barGtraspAA_;
	//Eigen::MatrixXd barGpinv = (barGtraspAA * barG + H).RegPseudoInverse(params.threshold, params.lambda, SVDvalues[SVDPARAM_PINV].mu,
	//				SVDvalues[SVDPARAM_PINV].flag);

	y_ = y_ + Q_ * barGpinv * barGtraspAA * W * (xdot - J * y_);
			//dout << tc::redL << "4.3" << std::endl;
	Q_ = Q_ * (I_ - barGpinv * barGtraspAA * barG);

};
