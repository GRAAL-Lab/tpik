#include "TPIK.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

TPIK::TPIK(int DoF){
	DoF_=DoF;
}
TPIK::~TPIK(){};
const Eigen::VectorXd& TPIK::GetY() const {
	return y_;
};



