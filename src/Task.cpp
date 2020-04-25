#include "tpik/Task.h"

namespace tpik {

Task::Task(const std::string ID, int taskSpace, int dof)
    : ID_(ID)
    , taskSpace_(taskSpace)
    , isActive_(true)
    , dof_(dof)

{
    Ai_.setZero(taskSpace_, taskSpace_);
    x_dot_.setZero(taskSpace_);
    J_.setZero(taskSpace_, dof_);
    Aexternal_.setIdentity(taskSpace_, taskSpace_);
}

Task::~Task() {}

void Task::Update()
{
    UpdateInternalActivationFunction();
    UpdateJacobian();
    UpdateReference();
    UpdateReferenceRate();
}
} // namespace tpik
