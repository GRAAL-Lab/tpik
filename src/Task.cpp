#include "tpik/Task.h"

namespace tpik
{

Task::Task(const std::string ID, int taskSpace, int DoF)
{
	ID_ = ID;
	taskSpace_ = taskSpace;
	DoF_ = DoF;
	Ai_.resize(taskSpace_, taskSpace_);
	Ai_.setZero();
	x_dot_.resize(taskSpace_);
	Ai_.setZero();
	J_.resize(taskSpace_, DoF_);
	Ai_.setZero();
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
