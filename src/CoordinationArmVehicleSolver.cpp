#include "tpik/CoordinationArmVehicleSolver.h"

namespace tpik {

CoordinationArmVehicleSolver::CoordinationArmVehicleSolver(const std::shared_ptr<ActionManager> actionManager, const std::shared_ptr<TPIK> tpik, const std::shared_ptr<PriorityLevel> vehiclePriorityLevel)
    : actionManager_(std::move(actionManager))
    , tpik_(std::move(tpik))
    , vehiclePriorityLevel_(std::move(vehiclePriorityLevel))
{
    hierarchy_ = actionManager_->GetHierarchy();
    vehiclePriorityLevel_->ActionTransitionActivation(1.0);
    hierarchyArm_.push_back(vehiclePriorityLevel_);
    hierarchyArm_.insert(hierarchyArm_.end(), hierarchy_.begin(), hierarchy_.end());
}

bool CoordinationArmVehicleSolver::SetAction(const std::string action, bool transition)
{
    return actionManager_->SetAction(action, transition);
}

const Eigen::VectorXd CoordinationArmVehicleSolver::ComputeDecoupledVelocities()
{
    actionManager_->ComputeActionTransitionActivation();
    tpik_->Reset();
    for (auto& plHierarchy : hierarchy_) {
        plHierarchy->Update();
        tpik_->ComputeVelocities(plHierarchy->Jacobian(), plHierarchy->ActivationFunction(), plHierarchy->ReferenceRate(), plHierarchy->RegularizationData());
    }
    Eigen::VectorXd yVehicle = tpik_->Velocities();
    tpik_->Reset();
    vehiclePriorityLevel_->Update();
    for (auto& plHierarchyArms : hierarchyArm_) {
        tpik_->ComputeVelocities(plHierarchyArms->Jacobian(), plHierarchyArms->ActivationFunction(), plHierarchyArms->ReferenceRate(), plHierarchyArms->RegularizationData());
    }
    Eigen::VectorXd y = rml::UnderJuxtapose(yVehicle.block(0, 0, 6, 1), tpik_->Velocities().block(6, 0, tpik_->Dof() - 6, 1));
    return y;
}
}
