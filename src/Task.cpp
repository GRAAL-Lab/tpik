#include "tpik/Task.h"
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace tpik
{

Task::Task(const std::string ID, int taskSpace, int DoF)
{
	ID_ = ID;
	taskSpace_ = taskSpace;
	DoF_ = DoF;
	Ai_.resize(taskSpace_, taskSpace_);
	x_dot_.resize(taskSpace_);
	J_.resize(taskSpace_, DoF_);
}

Task::~Task()
{
}

const Eigen::MatrixXd& Task::GetJacobian() const
{
	return J_;

}

const Eigen::MatrixXd& Task::GetInternalActivationFunction() const
{
	return Ai_;
}

const Eigen::VectorXd& Task::GetReference() const
{
	return x_dot_;

}

}
