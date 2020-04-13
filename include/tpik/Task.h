#ifndef __TASK_H__
#define __TASK_H__

#include "TPIKDefines.h"
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>

namespace tpik {
/**
 * @brief The Task pure virtual class.
 * @details Implementation of the abstract Task base class characterized by its
 * jacobian, activation function and reference. \n The class objects can be
 * gather in the PriorityLevel, organized in Action by using the
 * ActionManager.\n Once defined, a task priority can be solved by using the
 * Solver class. \n The derived classes must implement the following pure
 * virtual methods:
 *
 * * UpdateJacobian() where the user must update the class variable J_ which
 * stores the task jacobian;
 *
 * * UpdateInternalActivationFunction() where the user must update the class
 * variabel Ai_ which stores the task activation function;
 *
 * * UpdateReference() where the user must update the variable x_ which stores
 * the task reference,
 *
 * * Update() public method used to update the task variables, hence the
 * implementation of the previous pure virtual method must be called in order to
 * update all the class variables.
 *
 */
class Task {
public:
    /**
   * @brief Constructor of Task Class.
   * @details The Jacobian, reference and Internal Activation Function matrices
   * are pre-allocated and initialized to zeros.\n Jacobian (taskSpace x DoF)\n
   * Internal Activation Function (taskSpace x taskSpace)\n
   * Reference (taskSpace x 1)
   * @param[in] ID: Task ID;
   * @param[in] taskSpace: task space;
   * @param[in] DoF: Degrees of Freedom.
   */
    Task(const std::string ID, int taskSpace, int DoF);

    /**
   * @brief Default De-constructor of Task Class.
   */
    virtual ~Task();

    /**
   * @brief Method returning the Jacobian Matrix of the Task.
   * @return Jacobian Matrix.
   */
    const Eigen::MatrixXd& GetJacobian() const;
    /**
   * @brief Method returning the Internal Activation Function Matrix of the
   * Task.
   * @return Internal Activation Function.
   */
    const Eigen::MatrixXd& GetInternalActivationFunction() const;
    /**
   * @brief Method setting the external activation function for the rows i.e.
   * decide whether force the de activation of priority level rows
   * @param AeRows mask 0 if the rows wants to be deactivated, ones otherwise.
   */
    void SetExternalActivationFunction(Eigen::VectorXd Aexternal);

    /**
   * @brief Method returning the External Activation Function Matrix of the
   * Task.
   * @return External Activation Function.
   */
    const Eigen::MatrixXd& GetExternalActivationFunction() const;
    /**
   * @brief Method returning the Task Reference.
   * @return task Reference.
   */
    const Eigen::VectorXd& GetReference() const;
    /**
   * @brief Method returning the Task Degrees of Freedom.
   * @return Degrees of Freedom.
   */
    int GetDoF();
    /**
   * @brief Method returning the Task Space.
   * @return task Space.
   */
    int GetTaskSpace();
    /**
   * @brief Method returning the task enable boolean.
   * @return True if the task is active, false otherwise.
   * @note If the task is inactive, the internal activation function is set to 0
   * in the tpik::PriorityLevel.
   */
    bool GetIsActive();
    /**
   * @brief Method returning the task ID.
   * @return task ID.
   */
    const std::string GetID();
    /**
   * @brief Pure Virtual Method to be implemented by the derived classes to
   * update the task.
   */
    virtual void Update() = 0;
    /**
     * @brief Method returning the type of the task.
     */
    //    template <typename T>
    //    T GetType()
    //    {
    //        if (taskType_.inequalityType == InequalityTaskType::Decreasing || taskType_.inequalityType == InequalityTaskType::Increasing
    //            || taskType_.inequalityType == InequalityTaskType::Inbetween)
    //            return taskType_.cartesianType;
    //        else
    //            return taskType_.inequalityType;
    //    }

    /**
   * @brief Overload of the cout operator.
   */
    friend std::ostream& operator<<(std::ostream& os, Task const& task)
    {
        return os << "\033[1;37m"
                  << "Task ID " << task.ID_ << "\n"
                  << std::setprecision(4) << "\033[1;37m"
                  << "Internal Activation Function \n"
                  << "\033[0m" << task.Ai_ << "\n"
                  << "\033[1;37m"
                  << "Jacobian \n"
                  << "\033[0m" << task.J_ << "\n"
                  << "\033[1;37m"
                  << "Reference \n"
                  << "\033[0m" << task.x_dot_ << "\n";
    }
    /**
     * @brief Method to set the task parameters.
     * @param taskParameters.
     */
    virtual void SetTaskParameter(TaskParameter taskParameters) = 0;
    /**
     * @brief Method to set the gain task parameters.
     * @param double gain.
     */
    virtual void SetTaskParameter(double gain) = 0;
    /**
     * @brief Method to get the type of the task.
     */
    virtual TaskType GetTaskType() = 0;

protected:
    /**
   * @brief Pure Virtual Method to be implemented by the derived classes to
   * update the task internal activation function.
   */
    virtual void UpdateInternalActivationFunction() = 0;
    /**
   * @brief Pure Virtual Method to be implemented by the derived classes to
   * update the task reference.
   */
    virtual void UpdateReference() = 0;
    /**
   * @brief Pure Virtual Method to be implemented by the derived classes to
   * update the task Jacobian.
   *  */
    virtual void UpdateJacobian() = 0;

    std::string ID_{ "" }; //!< The task ID.
    Eigen::MatrixXd Ai_; //!< The internal activation function.
    Eigen::MatrixXd Aexternal_; //!< The activation function set by externely to
        //!< modify customly the Ai
    Eigen::MatrixXd J_; //!< The jacobian.
    Eigen::VectorXd x_dot_; //!< reference.
    int taskSpace_{ 0 }; //!< The task Space.
    int DoF_{ 0 }; //!< The degrees of freedom.
    bool isActive_{ true }; //!< The flag stating whether the task is active.
    TaskParameter taskParameter_{ 0, 0, 0 }; //!< The tpik::TaskParameter.
    BellShapedParameter bellshapedParam;
};
} // namespace tpik

#endif
