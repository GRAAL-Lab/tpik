#include "PriorityLevel.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>

PriorityLevel::PriorityLevel(std::string ID):taskNumber_(0),Ae_(0){
	ID_=ID;
}
PriorityLevel::PriorityLevel():taskNumber_(0),Ae_(0){
};
PriorityLevel::~PriorityLevel(){};

void PriorityLevel::AddTask(std::shared_ptr<Task> task){
	level_.push_back(task);
	taskNumber_=level_.size();

}


std::string PriorityLevel::GetID() const throw (PriorityLevelIndexException){
	if(ID_.empty()){
		throw PriorityLevelIndexException();
	}

	return ID_;

}
void PriorityLevel::SetID(std::string ID){
	ID_=ID;
};
const std::vector<std::shared_ptr<Task> > PriorityLevel:: GetLevel() const{
	return level_;
}

void PriorityLevel::SetExternalActivationFunction(double Ae){
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

//it is not const because it does not belong to the class (A_is not in the class)
Eigen::MatrixXd PriorityLevel::GetActivationFunction(){

	return Ae_*Ai_;
};

const Eigen::MatrixXd& PriorityLevel::GetInternalActivationFunction() const{
	return Ai_;
};

double PriorityLevel::GetExternalActivationFunction(){
	return Ae_;
};

const Eigen::VectorXd& PriorityLevel::GetReference() const{
	return x_dot_;
};

void PriorityLevel::UpdateJacobian(){
	J_=level_.at(0)->GetJacobian();
	for (auto& task: std::vector<std::shared_ptr<Task>> (level_.begin()+1,level_.end())){
		J_= rml::UnderJuxtapose(J_,task->GetJacobian());
}

};

void PriorityLevel::UpdateInternalActivationFunction(){
	Ai_=level_.at(0)->GetInternalActivationFunction();
	for (auto& task: std::vector<std::shared_ptr<Task>> (level_.begin()+1,level_.end())){
		Eigen::MatrixXd ANewTask= task->GetInternalActivationFunction();
		Eigen::MatrixXd Anew=rml::RightJuxtapose(Ai_,Eigen::MatrixXd::Zero(Ai_.rows(),ANewTask.cols()));
		Ai_=rml::UnderJuxtapose(Anew,rml::RightJuxtapose(Eigen::MatrixXd::Zero(ANewTask.rows()
				,Ai_.cols()),ANewTask));
	}


};

void PriorityLevel::UpdateReference(){
	x_dot_=level_.at(0)->GetReference();
	for (auto& task:std::vector<std::shared_ptr<Task>> (level_.begin()+1,level_.end())){
		x_dot_=rml::UnderJuxtapose(x_dot_,task->GetReference());
	}
};

void  PriorityLevel::UpdateAll(){
	PriorityLevel::UpdateJacobian();
	PriorityLevel::UpdateInternalActivationFunction();
	PriorityLevel::UpdateReference();

};

SVDParameters PriorityLevel::GetSVDParameter(){
	return svdParameters_;

};
