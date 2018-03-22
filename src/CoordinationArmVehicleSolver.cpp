#include "tpik/CoordinationArmVehicleSolver.h"

namespace tpik
{

CoordinationArmVehicleSolver::CoordinationArmVehicleSolver(std::shared_ptr<ActionManager> actionManager,
		std::shared_ptr<TPIK> tpik, std::shared_ptr<Task> vehicleTask, rml::SVDData vehicleTaskSVDParameter)
{
	actionManager_ = actionManager;
	hierarchy_ = actionManager_->GetHierarchy();
	tpik_ = tpik;
	auto vehiclePL = std::make_shared<PriorityLevel>(PriorityLevel("plVehicle"));
	vehiclePL_ = vehiclePL;
	vehicleTask_ = vehicleTask;
	vehiclePL_->AddTask(vehicleTask_);
	vehiclePL_->SetExternalActivationFunction(1.0);
	vehiclePL_->SetSVDParameters(vehicleTaskSVDParameter);
	hierarchyArm_.push_back(vehiclePL_);
	hierarchyArm_.insert(hierarchyArm_.end(), hierarchy_.begin(), hierarchy_.end());
}

void CoordinationArmVehicleSolver::SetAction(std::string action)
{
	actionManager_->SetAction(action);
}

void CoordinationArmVehicleSolver::SetTPIK(std::shared_ptr<TPIK> tpik)
{
	tpik_ = tpik;
}

const Eigen::VectorXd CoordinationArmVehicleSolver::ComputeDecoupledVelocities()
{

	actionManager_->ComputeExternalActivation();
	tpik_->Reset();
	for (auto& plHierarchy : hierarchy_) {
		plHierarchy->Update();
		tpik_->ComputeYSingleLevel(plHierarchy->GetJacobian(), plHierarchy->GetActivationFunction(),
				plHierarchy->GetReference(), plHierarchy->GetSVDParameter());
	}
	Eigen::VectorXd yVehicle = tpik_->GetY();
	tpik_->Reset();
	vehiclePL_->Update();
	for (auto& plHierarchyArms : hierarchyArm_) {
		tpik_->ComputeYSingleLevel(plHierarchyArms->GetJacobian(), plHierarchyArms->GetActivationFunction(),
				plHierarchyArms->GetReference(), plHierarchyArms->GetSVDParameter());
	}
	Eigen::VectorXd y = rml::UnderJuxtapose(yVehicle.block(0, 0, 6, 1),
			tpik_->GetY().block(6, 0, tpik_->GetDoF() - 6, 1));
	return y;
}

}
