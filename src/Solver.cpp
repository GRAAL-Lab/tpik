#include "tpik/Solver.h"
namespace tpik {

Solver::Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<iCAT> iCat)
    : actionManager_(std::move(actionManager))
    , iCat_(std::move(iCat))
{
    hierarchy_ = actionManager_->GetHierarchy();
}

void Solver::SetAction(const std::string action, bool transition)
{
    actionManager_->SetAction(action, transition);
}

const Eigen::VectorXd Solver::ComputeVelocities()
{
    actionManager_->ComputeActionTransitionActivation();
    iCat_->Reset();
    delta_y_.erase(delta_y_.begin(), delta_y_.end());
    Eigen::MatrixXd JMinimization;
    Eigen::MatrixXd AMinimization;
    Eigen::VectorXd XMinimization;
    rml::RegularizationData regularizationDataMinimization;

    AMinimization.Identity(iCat_->Dof(), iCat_->Dof());
    JMinimization.Identity(iCat_->Dof(), iCat_->Dof());
    XMinimization.Zero(iCat_->Dof());
    regularizationDataMinimization.params.lambda = 0.0;
    regularizationDataMinimization.params.threshold = 0.0;

    for (auto& priorityLevel : hierarchy_) {
        priorityLevel->Update();
        Eigen::MatrixXd J = priorityLevel->Jacobian();
        Eigen::MatrixXd A = priorityLevel->ActivationFunction();
        Eigen::VectorXd x_dot = priorityLevel->ReferenceRate();
        rml::RegularizationData regularizationData = priorityLevel->RegularizationData();
        iCat_->ComputeVelocities(J, A, x_dot, regularizationData);
        priorityLevel->DeltaY() = iCat_->DeltaY();
        delta_y_.push_back(iCat_->DeltaY());
    }
    iCat_->ComputeVelocities(JMinimization, AMinimization, XMinimization, regularizationDataMinimization);
    Eigen::VectorXd saturationMin;
    Eigen::VectorXd saturationMax;
    Eigen::VectorXd y = iCat_->Velocities();

    iCat_->GetSaturation(saturationMin, saturationMax);

    double min_factor = 1.0;
    for (int i = 0; i < y.size(); i++) {
        double factor = 1.0;
        if (y(i) > saturationMax(i)) {
            if (y(i) != 0.0) {
                factor = std::fabs(saturationMax(i) / y(i));
            }
        } else if (y(i) < saturationMin(i)) {
            if (y(i) != 0.0) {
                factor = std::fabs(saturationMin(i) / y(i));
            }
        }
        if (factor < min_factor) {
            min_factor = factor;
        }
    }

    for (int i = 0; i < y.size(); i++) {
        y(i) = y(i) * min_factor;
    }

    return y;
}
}
