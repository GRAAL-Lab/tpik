#ifndef __TASK_H__
#define __TASK_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include "TPIKDefines.h"

namespace tpik
{
/**
 * @brief Task class.
 * Implementation of the abstract Task base class. The derived classes must implement the pure virtual methods: UpdateJacobian, UpdateInternalActivationFunction
 * and UpdateReference.
 */
class Task
{
public:
	/**
	 * @brief Constructor of Task Class.
	 * The Jacobian, reference and Internal Activation Function matrices are pre-allocated and initialized to zeros.
	 * Jacobian (taskSpace x DoF)
	 * Internal Activation Function (taskSpace x taskSpace)
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
	 * @brief Method returning the Internal Activation Function Matrix of the Task.
	 * @return Internal Activation Function.
	 */
	const Eigen::MatrixXd& GetInternalActivationFunction() const;
	/**
	 * @brief Method returning the Task Reference.
	 * @return Task Reference.
	 */
	const Eigen::VectorXd& GetReference() const;
	/**
	 * @brief Method returning the Task Degrees of Freedom.
	 * @return Degrees of Freedom.
	 */
	int GetDoF();
	/**
	 * @brief Method returning the Task Space.
	 * @return Task Space.
	 */
	int GetTaskSpace();
	/**
	 * @brief Method returning the task enable boolean.
	 * @return True if the Task is active, false otherwise.
	 * @note If the task is inactive, the internal activation function is set to 0 in the tpik::PriorityLevel.
	 */
	bool GetIsActive();
	/**
	 * @brief Method returning the task ID.
	 * @return Task ID.
	 */
	const std::string GetID();
	/**
	 * @brief Pure Virtual Method to be implemented by the derived classes to update the Task.
	 */
	virtual void Update()=0;
	/**
	 * @brief Overload of the cout operator.
	 */
	friend std::ostream& operator <<(std::ostream& os, Task const& task)
	{
		return os << "\033[1;37m" << "Task ID " << task.ID_ << "\n" << std::setprecision(2) << "\033[1;37m"
				<< "Internal Activation Function \n" << "\033[0m" << task.Ai_ << "\n" << "\033[1;37m" << "Jacobian \n"
				<< "\033[0m" << task.J_ << "\n" << "\033[1;37m" << "Reference \n" << "\033[0m" << task.x_dot_ << "\n";
	}

protected:
	/**
	 * @brief Pure Virtual Method to be implemented by the derived classes to update the Task internal activation function.
	 */
	virtual void UpdateInternalActivationFunction()=0;
	/**
	 * @brief Pure Virtual Method to be implemented by the derived classes to update the Task reference.
	 */
	virtual void UpdateReference()=0;
	/**
	 * @brief Pure Virtual Method to be implemented by the derived classes to update the Task Jacobian.
	 *  */
	virtual void UpdateJacobian()=0;

	std::string ID_ { "" }; //!< The Task ID.
	Eigen::MatrixXd Ai_; //!< The internal activation function.
	Eigen::MatrixXd J_; //!< The jacobian.
	Eigen::VectorXd x_dot_; //!< reference.
	int taskSpace_{0}; //!< The task Space.
	int DoF_{0}; //!< The degrees of freedom.
	bool isActive_{false}; //!< The flag stating whether the task is active.

};
}

#endif
