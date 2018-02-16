#include "PriorityLevel.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

PriorityLevel::PriorityLevel(std::string ID):taskNumber_(0){
	ID_=ID;
}

PriorityLevel::~PriorityLevel(){};

void PriorityLevel::AddTask(std::shared_ptr<Task> task){
	level_.push_back(task);
	taskNumber_=level_.size();

}

std::string PriorityLevel::GetID(){
	return ID_;

}

const std::vector<std::shared_ptr<Task> > PriorityLevel:: GetLevel() const{
	return level_;
}

void PriorityLevel::SetExternalActivationFunction(Eigen::MatrixXd Ae){
	Ae_=Ae;
}

void PriorityLevel::SetSVDParameters(SVDParameters svdParameters){
	svdParameters_.lambda=svdParameters.lambda;
	svdParameters_.mu=svdParameters.mu;
	svdParameters_.threshold=svdParameters.threshold;
}

int PriorityLevel::GetNumberOfTask(){
	return level_.size();

};

const Eigen::MatrixXd& PriorityLevel::GetJacobian() const{
	return J_;
};

const Eigen::MatrixXd& PriorityLevel::GetActivationFunction() const{
	return A_;
};

const Eigen::MatrixXd& PriorityLevel::GetInternalActivationFunction() const{
	return Ai_;
};

const Eigen::MatrixXd& PriorityLevel::GetExternalActivationFunction() const{
	return Ae_;
};

const Eigen::VectorXd& PriorityLevel::GetReference() const{
	return x_dot_;
};

void PriorityLevel::UpdateJacobian(){
	for (auto& task:level_){
		Eigen::MatrixXd J=task->GetJacobian();
		//Juxtaposition
}

};

void PriorityLevel::UpdateActivationFunction(){
	for (auto& task:level_){
		Eigen::MatrixXd Ai= task->GetInternalActivationFunction();
	//Juxtaposition
	// multiply A_*Ae_
		}

};

void PriorityLevel::UpdateReference(){
	for (auto& task:level_){
		Eigen::MatrixXd x_dot= task->GetReference();
		//Juxtaposition
	}
};

void  PriorityLevel::UpdateAll(){
	PriorityLevel::UpdateJacobian();
	PriorityLevel::UpdateActivationFunction();
	PriorityLevel::UpdateReference();

};

