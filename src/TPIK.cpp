#include "TPIK.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

tpik::TPIK::TPIK(int DoF) {
	DoF_ = DoF;
	I_ = Eigen::MatrixXd::Identity(DoF_, DoF_);
	y_ = Eigen::VectorXd::Zero(DoF_);
	Q_ = I_;
}

tpik::TPIK::TPIK() {
	DoF_ = 0;
}

tpik::TPIK::~TPIK() {
}
;

const Eigen::VectorXd& tpik::TPIK::GetY() const {
	return y_;
}
;

void tpik::TPIK::Reset() {
	y_.setZero();
	Q_.setIdentity();

}

void tpik::TPIK::SetDoF(int DoF) {
	DoF_ = DoF;
}
;

int tpik::TPIK::GetDoF() {
	return DoF_;
}
;
