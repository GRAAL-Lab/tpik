#include "iCAT.h"
#include <iostream>
#include <vector>
#include <rml/RML.h>
#include <eigen3/Eigen/Dense>


tpik::iCAT::iCAT(int DoF):TPIK(DoF){
};

tpik::iCAT::iCAT():TPIK(){
};

void tpik::iCAT::SetDoF(int DoF){
	DoF_=DoF;
	I_=Eigen::MatrixXd::Identity(DoF_,DoF_);
	y_=Eigen::VectorXd::Zero(DoF_);
	Q_=I_;

}

tpik::iCAT::~iCAT(){};

void tpik::iCAT::ComputeYStep(Eigen::MatrixXd J,Eigen::MatrixXd Alpha,Eigen::VectorXd xdot,rml::SVDParameters svd) throw (TPIKMissingDoFInitializationException){
	if (DoF_==0){
		throw TPIKMissingDoFInitializationException();
	}
	Eigen::MatrixXd barG = J * Q_;
	Eigen::MatrixXd barGtraspAA = barG.transpose() * Alpha * Alpha;
	Eigen::MatrixXd T = (I_ - Q_).transpose() * (I_ - Q_);
	Eigen::MatrixXd H = barG.transpose() * (Eigen::MatrixXd::Identity(J.rows(),J.rows()) - Alpha) * Alpha * barG;

	Eigen::MatrixXd W = barG
				* rml::RegularizedPseudoInverse((Eigen::MatrixXd)(barGtraspAA * barG + T + H),svd)* barGtraspAA;
	Eigen::MatrixXd barGpinv = rml::RegularizedPseudoInverse((Eigen::MatrixXd)(barGtraspAA * barG + H),svd);
	y_ = y_ + Q_ * barGpinv * barGtraspAA * W * (xdot - J * y_);
	Q_ = Q_ * (I_ - barGpinv * barGtraspAA * barG);

};
