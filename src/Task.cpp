#include "tpik/Task.h"

namespace tpik {

Task::Task(const std::string ID, int taskSpace, int dof)
    : ID_(std::move(ID))
    , taskSpace_(taskSpace)
    , isActive_(true)
    , dof_(dof)

{
    Ai_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);
    x_dot_bar_ = Eigen::VectorXd::Zero(taskSpace_);
    Aexternal_ = Eigen::MatrixXd::Identity(taskSpace_, taskSpace_);
    J_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);
}

Task::~Task() {}

void Task::Update()
{
    UpdateInternalActivationFunction();
    UpdateJacobian();
    //UpdateReference();  //<- Solo nel Reactive, togliere da classe base e overridare con questa nella classe base Reactive <- TODO
    UpdateReferenceRate();
}
}
