#include "tpik/Solver.h"
namespace tpik {

Solver::Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik)
    : actionManager_(actionManager)
    , tpik_(tpik)
{
    hierarchy_ = actionManager_->GetHierarchy();
}

void Solver::SetTPIK(std::shared_ptr<TPIK> tpik) { tpik_ = tpik; }

void Solver::SetAction(std::string action, bool transition) { actionManager_->SetAction(action, transition); }

const Eigen::VectorXd& Solver::ComputeVelocities()
{
    actionManager_->ComputeExternalActivation();
    tpik_->Reset();
    delta_y.erase(delta_y.begin(), delta_y.end());
    Eigen::MatrixXd JMinimization;
    Eigen::MatrixXd AMinimization;
    Eigen::VectorXd XMinimization;
    rml::RegularizationData regularizationDataMinimization;

    AMinimization.Identity(tpik_->GetDoF(), tpik_->GetDoF());
    JMinimization.Identity(tpik_->GetDoF(), tpik_->GetDoF());
    XMinimization.Zero(tpik_->GetDoF());
    regularizationDataMinimization.params.lambda = 0.0;
    regularizationDataMinimization.params.threshold = 0.0;

    for (auto& priorityLevel : hierarchy_) {
        priorityLevel->Update();
        Eigen::MatrixXd J = priorityLevel->GetJacobian();
        Eigen::MatrixXd A = priorityLevel->GetActivationFunction();
        Eigen::MatrixXd x_dot = priorityLevel->GetReference();
        rml::RegularizationData regularizationData = priorityLevel->GetRegularizationData();
        tpik_->ComputeYSingleLevel(J, A, x_dot, regularizationData);
        priorityLevel->SetDeltaY(tpik_->GetDeltaY());
        delta_y.push_back(tpik_->GetDeltaY());
    }
    tpik_->ComputeYSingleLevel(JMinimization, AMinimization, XMinimization, regularizationDataMinimization);
    return tpik_->GetY();
}
std::vector<Eigen::VectorXd> Solver::GetDeltaYs() { return delta_y; }
}
