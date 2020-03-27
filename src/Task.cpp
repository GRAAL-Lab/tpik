#include "tpik/Task.h"

namespace tpik {

Task::Task(const std::string ID, int taskSpace, int DoF)
    : ID_(ID), taskSpace_(taskSpace), DoF_(DoF), isActive_(true) {
  //    Aexternal_.setZero(taskSpace_, taskSpace_);
  Ai_.setZero(taskSpace_, taskSpace_);
  x_dot_.setZero(taskSpace_);
  J_.setZero(taskSpace_, DoF_);
  Aexternal_.setZero(taskSpace_, taskSpace_);
}

Task::~Task() {}

const Eigen::MatrixXd &Task::GetJacobian() const { return J_; }

const Eigen::MatrixXd &Task::GetInternalActivationFunction() const {
  return Ai_;
}

 const Eigen::MatrixXd &Task::GetExternalActivationFunction() const {
  return Aexternal_;
}

 void Task::SetExternalActivationFunction(Eigen::VectorXd Aexternal) {
  if (Aexternal.size() != taskSpace_) {
    std::cout << "Ae" << Aexternal.transpose() << std::endl;
    std::cerr << "[TASK] ID: " << ID_
              << " Aerows wrong dimension, taskSpace = " << taskSpace_
              << "vector of size" << Aexternal.size() << "activating all rows"
              << std::endl;
  } else {
    Aexternal_ = Aexternal.asDiagonal();
  }
}
const Eigen::VectorXd &Task::GetReference() const { return x_dot_; }

int Task::GetDoF() { return DoF_; }

int Task::GetTaskSpace() { return taskSpace_; }

bool Task::GetIsActive() { return isActive_; }
const std::string Task::GetID() { return ID_; }
} // namespace tpik
