#include "tpik/Task.h"

namespace tpik
{

Task::Task(const std::string ID, int taskSpace, int DoF)
{
	ID_ = ID;
	taskSpace_ = taskSpace;
	DoF_ = DoF;
	Ai_.setZero(taskSpace_, taskSpace_);
	x_dot_.setZero(taskSpace_);
	J_.setZero(taskSpace_, DoF_);
	isActive_ = true;
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
int Task::GetDoF()
{
	return DoF_;

}

int Task::GetTaskSpace()
{
	return taskSpace_;
}

bool Task::GetIsActive()
{
	return isActive_;
}

}
