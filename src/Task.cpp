#include "Task.h"

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

Task::Task(TaskType taskType,const std::string ID):minBound_(0),maxBound_(0){
	type_=taskType;
	ID_=ID;
};
Task::~Task(){};

void Task::SetMinBound(double minBound){
	minBound_=minBound;

};

void Task::SetMaxBound(double maxBound){
	maxBound_=maxBound;

};

void Task::UpdateAll(){
	//Task::UpdateInternalActivationFunction();
	//Task::UpdateReference();
	//Task::UpdateJacobian();

};

void Task::SetTaskParameter(TaskParameter taskParameter){
	taskParameter_.TaskEnable=taskParameter.TaskEnable;
	if(type_==TaskType::InequalityLessThan|| type_==TaskType::InequalityInBetween){
		taskParameter_.max=taskParameter.max;
	}
	if(type_==TaskType::InequalityGreaterThan||type_==TaskType::InequalityInBetween){
		taskParameter_.min=taskParameter.min;
	}
}

const Eigen::MatrixXd& Task::GetJacobian() const {
	return J_;

}

const Eigen::MatrixXd& Task::GetInternalActivationFunction() const{
	return Ai_;
}

const Eigen::VectorXd& Task::GetReference() const{
	return x_dot_ ;

}
