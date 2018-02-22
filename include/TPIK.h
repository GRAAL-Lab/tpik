#ifndef __TPIK_H__
#define __TPIK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "PriorityLevel.h"
//TODO remove DoF, does not depend on it
class TPIK{
public:
	TPIK(int DoF);
	TPIK();
	virtual ~TPIK();
	virtual void ComputeYStep(Eigen::MatrixXd J,Eigen::MatrixXd Alpha,Eigen::VectorXd x_dot,rml::SVDParameters svd) {};
	const Eigen::VectorXd& GetY()const ;
	void Reset();
	void SetDoF(int DoF);
	int GetDoF();

protected:
	Eigen::VectorXd y_;
	Eigen::MatrixXd Q_;
	Eigen::MatrixXd I_;
	int DoF_;
};

#endif
