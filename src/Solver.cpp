#include "tpik/Solver.h"
namespace tpik
{

Solver::Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik): actionManager_(actionManager), tpik_(tpik)
{
	hierarchy_ = actionManager_->GetHierarchy();
}

void Solver::SetTPIK(std::shared_ptr<TPIK> tpik)
{
	tpik_ = tpik;
}

void Solver::SetAction(std::string action)
{
	actionManager_->SetAction(action);
}

const Eigen::VectorXd& Solver::ComputeVelocities() const
{
	actionManager_->ComputeExternalActivation();
	tpik_->Reset();
	for (auto& priorityLevel : hierarchy_) {
		priorityLevel->Update();
		Eigen::MatrixXd J = priorityLevel->GetJacobian();
		Eigen::MatrixXd A = priorityLevel->GetActivationFunction();
		Eigen::MatrixXd x_dot = priorityLevel->GetReference();
		rml::RegularizationData regularizationData = priorityLevel->GetRegularizationData();
		tpik_->ComputeYSingleLevel(J, A, x_dot, regularizationData);
	}
	return tpik_->GetY();
}

}
