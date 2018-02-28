#include "Solver.h"
#include <iostream>
#include <vector>
#include <rml/RML.h>
#include <eigen3/Eigen/Dense>

namespace tpik {
Solver::Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik) {
	actionManager_ = actionManager;
	hierarchy_ = actionManager_->GetHierarchy();
	tpik_ = tpik;
}

Solver::Solver() {
}

void Solver::SetActionManager(std::shared_ptr<ActionManager> actionManager) {
	actionManager_ = actionManager;
	hierarchy_ = actionManager_->GetHierarchy();
}

void Solver::SetTPIK(std::shared_ptr<TPIK> tpik) {
	tpik_ = tpik;
}

void Solver::SetAction(std::string action) throw (SolverNotInitializationException) {
	if (actionManager_ == nullptr) {
		throw(SolverNotInitializationException());
	}
	actionManager_->SetAction(action);
}

const Eigen::VectorXd Solver::ComputeVelocities() throw (SolverNotInitializationException) {
	if (actionManager_ == nullptr || tpik_ == nullptr) {
		throw SolverNotInitializationException();
	}
	actionManager_->ComputeExternalActivation();
	tpik_->Reset();
	for (auto& priorityLevel : hierarchy_) {
		Eigen::MatrixXd J = priorityLevel->GetJacobian();
		Eigen::MatrixXd A = priorityLevel->GetActivationFunction();
		Eigen::MatrixXd x_dot = priorityLevel->GetReference();
		rml::SVDParameters svd = priorityLevel->GetSVDParameter();
		//rml::PrintMatrix(A,"Act Func");
		tpik_->ComputeYStep(J, A, x_dot, svd);
		futils::PrettyPrint(tpik_->GetY(), "Y Step");
	}
	return tpik_->GetY();
}

}
