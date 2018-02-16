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
		 	for(auto& priorityLevelHierarchy:hierarchy_){
		 		Eigen::MatrixXd J=priorityLevelHierarchy->GetJacobian();
		 		Eigen::MatrixXd A=priorityLevelHierarchy->GetActivationFunction();
		 		Eigen::MatrixXd x_dot=priorityLevelHierarchy->GetReference();
		 		tpik_->ComputeYStep(J,A,x_dot);
		 	}
         return tpik_->GetY();
};
