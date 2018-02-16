#ifndef __TPIK_H__
#define __TPIK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "PriorityLevel.h"

class TPIK{
public:
	TPIK(int DoF);
	virtual ~TPIK();
	virtual void ComputeYStep(Eigen::MatrixXd J,Eigen::MatrixXd Alpha,Eigen::VectorXd x_dot) {};
	const Eigen::VectorXd& GetY()const ;

protected:
	Eigen::VectorXd y_;
	Eigen::MatrixXd Q_;
	int DoF_;
};

#endif
