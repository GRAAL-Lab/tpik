#include "tpik/PriorityLevel.h"

namespace tpik {

PriorityLevel::PriorityLevel(const std::string ID)
    : taskNumber_(0)
    , Ae_(0)
    , ID_(ID)
    , priorityLevelSpace_(0)
{
}

PriorityLevel::~PriorityLevel() {}

void PriorityLevel::AddTask(std::shared_ptr<Task> task)
{
    level_.push_back(task);
    taskNumber_ = level_.size();
    priorityLevelSpace_ += task->GetTaskSpace();
    Aerows_ = Eigen::MatrixXd::Identity(priorityLevelSpace_, priorityLevelSpace_);
}

std::string PriorityLevel::GetID() const { return ID_; }

void PriorityLevel::SetExternalActivationFunctionRows(Eigen::VectorXd AeRows)
{
    if (AeRows.size() != priorityLevelSpace_) {
        std::cout <<"Ae" <<AeRows.transpose()<<std::endl;
        std::cerr << "[PRIORITY LEVEL ] ID: " << ID_
                  << " Aerows wrong dimension, priorityLevelSpace = " << priorityLevelSpace_ << "vector of size"<<AeRows.size()<<"activating all rows"
                  << std::endl;
    }
    else {
        Aerows_ = AeRows.asDiagonal();
    }
}

void PriorityLevel::UpdateJacobian()
{
    J_ = level_.at(0)->GetJacobian();
    for (auto& task : std::vector<std::shared_ptr<Task> >(level_.begin() + 1, level_.end())) {
        J_ = rml::UnderJuxtapose(J_, task->GetJacobian());
    }
}

void PriorityLevel::UpdateInternalActivationFunction()
{
    bool isAiInitialized = false;
    for (auto& task : level_) {
        if (isAiInitialized) {
            Eigen::MatrixXd ANewTask;

            if (task->GetIsActive()) {
                ANewTask = task->GetInternalActivationFunction();

            } else {
                ANewTask = Eigen::MatrixXd::Zero(task->GetTaskSpace(), task->GetTaskSpace());
            }
            Eigen::MatrixXd Anew = rml::RightJuxtapose(Ai_, Eigen::MatrixXd::Zero(Ai_.rows(), ANewTask.cols()));
            Ai_ = rml::UnderJuxtapose(
                Anew, rml::RightJuxtapose(Eigen::MatrixXd::Zero(ANewTask.rows(), Ai_.cols()), ANewTask));

        } else {
            if (task->GetIsActive()) {
                Ai_ = task->GetInternalActivationFunction();
                isAiInitialized = true;

            } else {
                Ai_ = Eigen::MatrixXd::Zero(task->GetTaskSpace(), task->GetTaskSpace());
                isAiInitialized = true;
            }
        }
    }
}

void PriorityLevel::UpdateReference()
{
    x_dot_ = level_.at(0)->GetReference();
    for (auto& task : std::vector<std::shared_ptr<Task> >(level_.begin() + 1, level_.end())) {
        x_dot_ = rml::UnderJuxtapose(x_dot_, task->GetReference());
    }
}

void PriorityLevel::SetDeltaY(Eigen::VectorXd deltaY) { deltaY_ = deltaY; }

Eigen::VectorXd PriorityLevel::GetDeltaY() { return deltaY_; }

void PriorityLevel::Update()
{
    UpdateJacobian();
    UpdateInternalActivationFunction();
    UpdateReference();
}

void PriorityLevel::SetExternalActivationFunction(double Ae) { Ae_ = Ae; }

void PriorityLevel::SetRegularizationData(rml::RegularizationData regularizationData)
{
    regularizationData_ = regularizationData;
}
const Eigen::MatrixXd& PriorityLevel::GetJacobian() const { return J_; }

Eigen::MatrixXd PriorityLevel::GetActivationFunction()
{

    return Ae_ * Ai_ * Aerows_;
    // return  Ai_;
}

const Eigen::MatrixXd& PriorityLevel::GetInternalActivationFunction() const { return Ai_; }

const double PriorityLevel::GetExternalActivationFunction() { return Ae_; }

const Eigen::VectorXd& PriorityLevel::GetReference() const { return x_dot_; }

int PriorityLevel::GetNumberOfTask() { return level_.size(); }

const std::vector<std::shared_ptr<Task> > PriorityLevel::GetLevel() const { return level_; }

const rml::RegularizationData& PriorityLevel::GetRegularizationData() { return regularizationData_; }
}
