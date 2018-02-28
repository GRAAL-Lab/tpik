#include "Task.h"
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace tpik {
Task::Task(const std::string ID){
	ID_ = ID;
}


Task::~Task() {
}

void Task::SetID(const std::string ID) {
	ID_ = ID;
}

const Eigen::MatrixXd& Task::GetJacobian() const {
	return J_;

}

const Eigen::MatrixXd& Task::GetInternalActivationFunction() const {
	return Ai_;
}

const Eigen::VectorXd& Task::GetReference() const {
	return x_dot_;

}

}
