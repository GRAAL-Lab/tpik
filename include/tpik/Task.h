#ifndef TASK_H
#define TASK_H

#include "TPIKDefines.h"
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>

namespace tpik {
/*
 * @brief The Task virtual class.
 *
 * @details Implementation of the abstract Task base class characterized by its jacobian, activation functions and reference. The class objects can be
 * gather in the PriorityLevel, organized in Action by using the ActionManager. Once defined, a task priority can be solved by using the
 * Solver class. The derived classes must implement the following pure virtual methods:
 *
 * - UpdateJacobian() where the user must update the class variable J_ which stores the task jacobian;
 *
 * - UpdateInternalActivationFunction() where the user must update the class variabel Ai_ which stores the task activation function;
 *
 * - UpdateReferenceRate() where the user must update the variable x_dot which stores the task reference rate,
 *
 * - UpdateReference() where the user must update the variable x_ which stores the task reference,
 *
 * - Update() public method used to update the task variables.
 */
class Task {
public:
    /*
    * @brief Constructor of Task Class.
    * @details The Jacobian, reference and Internal Activation Function matrices are pre-allocated:
    * - Jacobian (taskSpace x DoF)
    * - Internal Activation Function (taskSpace x taskSpace)
    * - External Activation Function (taskSpace x taskSpace)
    * - ReferenceRate x_dot (taskSpace x 1)
    * @param[in] ID: Task ID;
    * @param[in] taskSpace: task space;
    * @param[in] dof: Degrees of Freedom.
    */
    Task(const std::string ID, int taskSpace, int dof);
    /*
    * @brief Default De-constructor of Task Class.
    */
    virtual ~Task();
    /*
    * @brief Method returning the Jacobian Matrix of the Task.
    * @return Jacobian Matrix.
    */
    auto Jacobian() const -> const Eigen::MatrixXd& { return J_; }
    /*
    * @brief Method returning the Internal Activation Function Matrix of the Task.
    * @return Internal Activation Function.
    */
    auto InternalActivationFunction() const -> const Eigen::MatrixXd& { return Ai_; }
    /*
    * @brief Methods setting the external activation function
    * @detrails This function must be use to active customly a task variable by setting to one the corresponding diagonal matrix selement
    */
    auto ExternalActivationFunction() -> Eigen::MatrixXd& { return Aexternal_; }
    /*
    * @brief Methods getting the external activation function
    * @param Internal Activation Function.
    */
    auto ExternalActivationFunction() const -> const Eigen::MatrixXd& { return Aexternal_; }
    /*
    * @brief Method returning the Task Reference rate
    * @return task Reference rate.
    */
    auto ReferenceRate() const -> const Eigen::VectorXd& { return x_dot_; }
    /*
    * @brief Method returning the Task Degrees of Freedom.
    * @return Degrees of Freedom.
    */
    auto DoF() const -> int { return dof_; }
    /*
    * @brief Method returning the Task Space.
    * @return task Space.
    */
    auto TaskSpace() const -> int { return taskSpace_; }
    /*
    * @brief Method returning the task enable boolean.
    * @return True if the task is active, false otherwise.
    * @note If the task is inactive, the internal activation function is set to 0 in the tpik::PriorityLevel.
    */
    auto IsActive() const -> bool { return isActive_; }
    /*
    * @brief Method returning the task ID.
    * @return task ID.
    */
    auto ID() const -> const std::string& { return ID_; }
    /*
    * @brief Pure Virtual Method to be implemented by the derived classes to update the task.
    */
    virtual void Update();
    /*
    * @brief Pure Virtual Method to config from file the task
    * @details This method allows to read the task params form file using libconfig formalism. Alternately, it can be used the setting methods of the derived classes to acquired the params
    */
    virtual void ConfigFromFile(libconfig::Config& confObj) = 0;
    /*
    * @brief Overload of the cout operator.
    */
    friend std::ostream& operator<<(std::ostream& os, Task const& task)
    {
        return os << "\033[1;37m"
                  << "Task ID " << task.ID_ << "\n"
                  << std::setprecision(4) << "Internal Activation Function \n"
                  << "\033[0m" << task.Ai_ << "\n"
                  << "External Activation Function\n"
                  << "\033[0m" << task.Aexternal_ << "\n"
                  << "\033[1;37m"
                  << "Jacobian \n"
                  << "\033[0m" << task.J_ << "\n"
                  << "\033[1;37m"
                  << "Reference Rate \n"
                  << "\033[0m" << task.x_dot_ << "\n";
    }

protected:
    /*
    * @brief Pure Virtual Method to be implemented by the derived classes to update the task internal activation function.
    */
    virtual void UpdateInternalActivationFunction() = 0;
    /*
    * @brief Pure Virtual Method to be implemented by the derived classes to update the task reference rate.
    */
    virtual void UpdateReferenceRate() = 0;
    /*
    * @brief Pure Virtual Method to be implemented by the derived classes to update the task reference.
    */
    virtual void UpdateReference() = 0;
    /*
    * @brief Pure Virtual Method to be implemented by the derived classes to update the task Jacobian.
    */
    virtual void UpdateJacobian() = 0;

    std::string ID_; // The task ID.
    Eigen::MatrixXd Ai_; // The internal activation function.
    Eigen::MatrixXd Aexternal_; // The activation function set externely to modify customly the Ai
    Eigen::MatrixXd J_; // The jacobian.
    Eigen::VectorXd x_dot_; // reference rate.
    int taskSpace_; // The task Space.
    bool isActive_; // The flag stating whether the task is active.
    int dof_; // The degrees of freedom.
};
} // namespace tpik

#endif
