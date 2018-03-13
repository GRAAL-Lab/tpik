#include "tpik/Solver.h"
#include <iostream>
#include <vector>
#include <rml/RML.h>
#include <eigen3/Eigen/Dense>

namespace tpik
{

Solver::Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik)
{
	actionManager_ = actionManager;
	hierarchy_ = actionManager_->GetHierarchy();
	tpik_ = tpik;
}

void Solver::SetTPIK(std::shared_ptr<TPIK> tpik)
{
	tpik_ = tpik;
}

void Solver::SetAction(std::string action)
{
	actionManager_->SetAction(action);
}

const Eigen::VectorXd Solver::ComputeVelocities()
{
	actionManager_->ComputeExternalActivation();
	tpik_->Reset();
	for (auto& priorityLevel : hierarchy_) {
		priorityLevel->Update();
		Eigen::MatrixXd J = priorityLevel->GetJacobian();
		Eigen::MatrixXd A = priorityLevel->GetActivationFunction();
		Eigen::MatrixXd x_dot = priorityLevel->GetReference();
		rml::SVDParameters svd = priorityLevel->GetSVDParameter();
		tpik_->ComputeYSingleLevel(J, A, x_dot, svd);
	}
	return tpik_->GetY();
}

}
