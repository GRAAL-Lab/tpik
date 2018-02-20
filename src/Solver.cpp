#include "Solver.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

Solver::Solver(std::shared_ptr<ActionManager> actionManager,std::shared_ptr<TPIK> tpik){
		actionManager_=actionManager;
	    hierarchy_=actionManager_->GetHierarchy();
		tpik_=tpik;
};
void  Solver::SetAction(std::string action){
		actionManager_->SetAction(action);
};
const Eigen::VectorXd Solver::ComputeVelocities(){
		 actionManager_->ComputeExternalActivation();
		 tpik_->Reset();
		 	for(auto& priorityLevel:hierarchy_){
		 		Eigen::MatrixXd J=priorityLevel->GetJacobian();
		 		Eigen::MatrixXd A=priorityLevel->GetActivationFunction();
		 		Eigen::MatrixXd x_dot=priorityLevel->GetReference();
		 		SVDParameters svd=priorityLevel->GetSVDParameter();
		 		tpik_->ComputeYStep(J,A,x_dot,svd);
		 	}
         return tpik_->GetY();
};
