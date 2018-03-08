#include "CoordinationArmVehicleSolver.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>
#include <memory>

namespace tpik {

CoordinationArmVehicleSolver::CoordinationArmVehicleSolver(std::shared_ptr<tpik::ActionManager> actionManager, std::shared_ptr<tpik::TPIK> tpik,
		std::shared_ptr<Task> vehicleTask,rml::SVDParameters vehicleTaskSVDParameter) {
	actionManager_ = actionManager;
	hierarchy_ = actionManager_->GetHierarchy();
	tpik_ = tpik;
	auto vehiclePL = std::make_shared<tpik::PriorityLevel>(tpik::PriorityLevel("plVehicle"));
	vehiclePL_ = vehiclePL;
	vehicleTask_ = vehicleTask;
	vehiclePL_->AddTask(vehicleTask_);
	vehiclePL_->SetExternalActivationFunction(1.0);
	vehiclePL_->SetSVDParameters(vehicleTaskSVDParameter);
	hierarchyArm_.push_back(vehiclePL_);
	hierarchyArm_.insert(hierarchyArm_.end(), hierarchy_.begin(), hierarchy_.end());
}

void CoordinationArmVehicleSolver::SetAction(std::string action) {
	actionManager_->SetAction(action);
}

void CoordinationArmVehicleSolver::SetActionManager(std::shared_ptr<tpik::ActionManager> actionManager) {
	actionManager_ = actionManager;
	hierarchy_ = actionManager_->GetHierarchy();
	auto plVehicle = std::make_shared<tpik::PriorityLevel>(tpik::PriorityLevel("plVehicle"));
	plVehicle->AddTask(vehicleTask_);
	hierarchyArm_.push_back(plVehicle);
	hierarchyArm_.insert(hierarchyArm_.end(), hierarchy_.begin(), hierarchy_.end());
}

void CoordinationArmVehicleSolver::SetVehicleTask(std::shared_ptr<Task> vehicleTask) {
	vehicleTask_ = vehicleTask;
}

void CoordinationArmVehicleSolver::SetTPIK(std::shared_ptr<tpik::TPIK> tpik) {
	tpik_ = tpik;
}


const Eigen::VectorXd CoordinationArmVehicleSolver::ComputeDecoupledVelocities() {

	actionManager_->ComputeExternalActivation();
	tpik_->Reset();
	for (auto& plHierarchy : hierarchy_) {
		plHierarchy->UpdateAll();
		tpik_->ComputeYSingleLevel(plHierarchy->GetJacobian(), plHierarchy->GetActivationFunction(),
				plHierarchy->GetReference(), plHierarchy->GetSVDParameter());
	}

	Eigen::VectorXd yVehicle = tpik_->GetY();
	tpik_->Reset();
	vehiclePL_->UpdateAll();

	for (auto& plHierarchyArms : hierarchyArm_) {
		tpik_->ComputeYSingleLevel(plHierarchyArms->GetJacobian(), plHierarchyArms->GetActivationFunction(),
				plHierarchyArms->GetReference(), plHierarchyArms->GetSVDParameter());
	}
	Eigen::VectorXd y = rml::UnderJuxtapose(yVehicle.block(0, 0, 6, 1), tpik_->GetY().block(6, 0, 7, 1));

	return y;
}

}
