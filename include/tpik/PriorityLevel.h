#ifndef __PRIORITYLEVEL_H__
#define __PRIORITYLEVEL_H__

#include "Task.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RML.h>
#include <vector>

namespace tpik {
/**
 * @brief PriorityLevel class.
 * @details Implementation of the PriorityLevel Class. Starting form a vector of
 * tpik::Task which have the same priority, it computes the Jacobian, internal
 * activation function and reference by juxtaposing the related task matrices.
 * Methods to set the external Activation Function are provided. It is assumed
 * that all the Tasks have the same behavior wrt the external activation
 * function (hence either they are all present or none of them are present in an
 * action). The overall activation function is computed as product between the
 * internal and external activation functions.
 */
class PriorityLevel {
public:
  /**
   * @brief Constructor of PriorityLevel Class.
   * @param[in] ID Priority Level ID.
   */
  PriorityLevel(const std::string ID);

  /**
   * @brief Default De-constructor of PriorityLevel Class.
   */
  ~PriorityLevel();
  /**
   * @brief Method which adds a tpik::Task to the PriorityLevel.
   * @param[in] task std::shared_ptr to the tpik::Task to be added.
   */
  void AddTask(std::shared_ptr<Task> task);
  /**
   * @brief Method returning the PriorityLevel ID.
   * @return PriorityLevel ID.
   */
  std::string GetID() const;
  /**
   * @brief Method which updates the PriorityLevel Jacobian, Internal
   * ActivationFunction and Reference.
   */
  void Update();
  /**
   * @brief Method setting the external activation function of the
   * PriorityLevel, it is supposed that such value is equal for all the priority
   * Level tasks.
   *
   */
  void SetActionTransitionActivation(double ActionTransitionA);
  /**
   * @brief Method setting the PriorityLevel regularization data.
   * @param[in] regularizationData rml::RegularizationData struct.
   */
  void SetRegularizationData(rml::RegularizationData regularizationData);

  /**
   * @brief Method Returning the PriorityLevel Jacobian.
   * @return PriorityLevel Jacobian.
   */
  const Eigen::MatrixXd &GetJacobian() const;
  /**
   * @brief Method Returning the PriorityLevel Activation Function computed as
   * follows : Ae*Ai.
   * @return Activation Function.
   */
  Eigen::MatrixXd GetActivationFunction();
  /**
   * @brief Method returning the PriorityLevel Internal Activation Function.
   * @return Internal Activation Function.
   */
  const Eigen::MatrixXd &GetInternalActivationFunction() const;
  /**
   * @brief Method returning the PriorityLevel action transition Activation
   * Function.
   * @return Action transition Activation Function.
   */
  double GetActionTransitionActivation();
  /**
   * @brief Method returning the PriorityLevel Reference.
   * @return PriorityLevel Reference.
   */
  const Eigen::VectorXd &GetReference() const;
  /**
   * @brief Method returning the PriorityLevel number of tpik::Task.
   * @return Number of Task.
   */
  int GetNumberOfTask();
  /**
   * @brief Method returning the PriorityLevel Tasks as vector of shared_ptr to
   * tpik::Task.
   * @return Vector of shared_ptr of tpik::Task.
   */
  const std::vector<std::shared_ptr<Task>> GetLevel() const;
  /**
   * @brief Function Returning the PriorityLevel SVDParameters.
   * @return PriorityLevel rml::SVDParameter.
   */
  const rml::RegularizationData &GetRegularizationData();

  /**
   * @brief Method used to set the last increment computed for the priority
   * level.
   */
  void SetDeltaY(Eigen::VectorXd deltaY);
  /**
   * @brief Method used to get the last increment computed for the priority
   * level.
   * @return delta y
   */
  Eigen::VectorXd GetDeltaY();
  /**
   * @brief Function overloading the cout operator
   */
  friend std::ostream &operator<<(std::ostream &os,
                                  PriorityLevel const &priorityLevel) {
    return os << "\033[1;37m"
              << "PriorityLevel ID " << priorityLevel.ID_ << "\n"
              << std::setprecision(4) << "\033[1;37m"
              << "Internal Activation Function \n"
              << "\033[0m" << priorityLevel.Ai_ << "\n"
              << "\033[1;37m"
              << "External Activation Function "
              << "\033[0m" << priorityLevel.actionTransitionA_ << "\n"
              << "\033[1;37m"
              << "Jacobian \n"
              << "\033[0m" << priorityLevel.J_ << "\n"
              << "\033[1;37m"
              << "Reference \n"
              << "\033[0m" << priorityLevel.x_dot_ << "\n"
              << "\033[1;37m"
              << "svdParameters\nThrehsold "
              << "\033[0m" << priorityLevel.regularizationData_.params.threshold
              << "\n"
              << "\033[1;37m"
              << "lambda "
              << "\033[0m" << priorityLevel.regularizationData_.params.lambda
              << "\n";
  }

private:
  /**
   * @brief Method which juxtaposes the Task Jacobians to obtain the priority
   * level Jacobian.
   */
  void UpdateJacobian();
  /**
   * @brief Method which juxtaposes the Task Internal Activation Functions to
   * obtain the priority level Internal Activation Function.
   */
  void UpdateInternalActivationFunction();
  /**
   * @brief Method which juxtaposes the Task References to obtain the priority
   * level References.
   */
  void UpdateReference();

  std::vector<std::shared_ptr<Task>>
      level_; //!< The vector containing the std::shared_ptr to tpik::Task
              //!< objects.
  std::string ID_{" "}; //!< The PriorityLevel ID.
  Eigen::MatrixXd Ai_;  //!< The internal activation function.
  Eigen::MatrixXd
      Aextern_; //!< The activation function set by externely to modify customly

  Eigen::MatrixXd J_;     //!< The jacobian.
  Eigen::VectorXd x_dot_; //!< The reference.
  double actionTransitionA_{
      0.0}; //!< The  action transition  activation function.
  unsigned long taskNumber_{0}; //!< The priority level number of tasks.
  rml::RegularizationData
      regularizationData_; //!< The rml::RegularizationData struct, used to
                           //!< compute the regularized pseudoinverse.
  Eigen::VectorXd
      deltaY_; //!< The last delta y comptued for the priority level.
  int priorityLevelSpace_;
};
} // namespace tpik

#endif
