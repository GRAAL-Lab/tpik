#ifndef __TPIK_H__
#define __TPIK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "PriorityLevel.h"
//TODO remove DoF, does not depend on it
namespace tpik{
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
	friend std::ostream& operator <<(std::ostream& os, TPIK const& tpik){
			return os<< "\033[1;37m"<<"TPIK"<<"\n"<<std::setprecision(2)
					  << "\033[1;37m"<<"Y \n"<<"\033[0m"<<tpik.y_<<"\n"
					  << "\033[1;37m"<<"Q \n"<<"\033[0m"<<tpik.Q_<<"\n";};


protected:
	Eigen::VectorXd y_;
	Eigen::MatrixXd Q_;
	Eigen::MatrixXd I_;
	int DoF_;
};
}

#endif
