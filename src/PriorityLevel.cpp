#include "tpik/PriorityLevel.h"

namespace tpik {

PriorityLevel::PriorityLevel(const std::string ID)
    : ID_{ std::move(ID) }
    , actionTransitionA_{ 0.0 }
    , priorityLevelSpace_{ 0 }
{
}

PriorityLevel::~PriorityLevel() {}

void PriorityLevel::AddTask(const std::shared_ptr<Task> task)
{
    level_.push_back(task);
    priorityLevelSpace_ += task->TaskSpace();
    Aextern_ = Eigen::MatrixXd::Zero(priorityLevelSpace_, priorityLevelSpace_);
    Ai_ = Eigen::MatrixXd::Zero(priorityLevelSpace_, priorityLevelSpace_);
    A_ = Eigen::MatrixXd::Zero(priorityLevelSpace_, priorityLevelSpace_);
}

void PriorityLevel::UpdateJacobian()
{
    J_ = level_.at(0)->Jacobian();
    for (auto& task : std::vector<std::shared_ptr<Task>>(level_.begin() + 1, level_.end())) {
        J_ = rml::UnderJuxtapose(J_, task->Jacobian());
    }
}

void PriorityLevel::UpdateActivationFunction()
{
    int taskSpacei = 0;
    int lastTaskSpace = priorityLevelSpace_;
    Eigen::MatrixXd AexternalBloc, AinternalBlock;

    for (auto& task : level_) {
        taskSpacei = task->TaskSpace();

        AexternalBloc = task->ExternalActivationFunction();
        AinternalBlock = task->InternalActivationFunction();

        Aextern_.block(priorityLevelSpace_ - lastTaskSpace, priorityLevelSpace_ - lastTaskSpace, taskSpacei, taskSpacei) = AexternalBloc;

        Ai_.block(priorityLevelSpace_ - lastTaskSpace, priorityLevelSpace_ - lastTaskSpace, taskSpacei, taskSpacei) = AinternalBlock;

        lastTaskSpace -= taskSpacei;
    }

    A_ = actionTransitionA_ * Ai_ * Aextern_;
}

void PriorityLevel::UpdateReferenceRate()
{
    x_dot_ = level_.at(0)->ReferenceRate();

    for (auto& task : std::vector<std::shared_ptr<Task>>(level_.begin() + 1, level_.end())) {
        x_dot_ = rml::UnderJuxtapose(x_dot_, task->ReferenceRate());
    }
}

void PriorityLevel::Update()
{
    UpdateJacobian();
    UpdateActivationFunction();
    UpdateReferenceRate();
}

} // namespace tpik
