#ifndef __TASK_H__
#define __TASK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "TPIKDefines.h"

namespace tpik {

class Task {
public:
	/**
	 * @brief Constructor of Task Class
	 * @param[in] type: Task type: {Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween}
	 * @param[in] ID: Task ID
	 *  */
	Task(TaskType type, const std::string ID); // ID is set by the user in order to uniquely identify the task
	/**
	 * @brief Constructor of Task Class
	 * @param[in] type: Task type: {Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween}
	 *  */
	Task(TaskType type);
	/**
	 * @brief Default Deconstructor of Task Class
	 *  */
	virtual ~Task();
	/**
	 * @brief Set Task ID
	 * @param[in] ID : Task ID.
	 *  */
	void SetID(const std::string ID);
	/**
	 * @brief Set Task Minimum bound for Inequality task to define the interval in which the task must be active
	 * @param[in] ID : minBound: minimum interval value .
	 *  */
	void SetMinBound(double minBound);

	/**
	 * @brief Set Task Maximum bound for Inequality task to define the interval in which the task must be active
	 * @param[in] ID : maxBound: maximum interval value .
	 *  */
	void SetMaxBound(double maxBound);
	/**
	 * @brief Function returning the Jacobian Matrix of the Task
	 *  */
	const Eigen::MatrixXd& GetJacobian() const; // in this way from the main it will be not possible to change both the pointer and the matrix
	/**
	 * @brief Function returning the Internal Activation Function Matrix of the Task
	 *  */
	const Eigen::MatrixXd& GetInternalActivationFunction() const;
	/**
	 * @brief Function returning the Task Reference
	 *  */
	const Eigen::VectorXd& GetReference() const;
	/**
	 * @brief Set the Task Parameter (gain, bell shaped function parameter, task enable boolean)
	 * @param[in] taskParameter: Task Parameter struct
	 *  */
	void SetTaskParameter(TaskParameter taskParameter);
	/**
	 * @brief Pure Virtual Function to be implemented by the derived class to update the task internal activation function
	 *  */
	virtual void UpdateInternalActivationFunction()=0;
	/**
	 * @brief Pure Virtual Function to be implemented by the derived class to update the task reference
	 *  */
	virtual void UpdateReference()=0;
	/**
	 * @brief Pure Virtual Function to be implemented by the derived class to update the task jacobian
	 *  */
	virtual void UpdateJacobian()=0;
	/**
	 * @brief Overload of the cout operator
	 *  */
	friend std::ostream& operator <<(std::ostream& os, Task const& task) {
		return os << "\033[1;37m" << "Task ID " << task.ID_ << "\n"
				<< std::setprecision(2) << "\033[1;37m"
				<< "Internal Activation Function \n" << "\033[0m" << task.Ai_
				<< "\n" << "\033[1;37m" << "Jacobian \n" << "\033[0m" << task.J_
				<< "\n" << "\033[1;37m" << "Reference \n" << "\033[0m"
				<< task.x_dot_ << "\n" << "\033[1;37m" << "minBound \n"
				<< "\033[0m" << task.minBound_ << "\n" << "\033[1;37m"
				<< "maxBound \n" << "\033[0m" << task.maxBound_ << "\n"
				<< "\033[1;37m" << "TaskParameter (Gain TaskEnable) \n"
				<< "\033[0m" << task.taskParameter_.gain << "	"
				<< task.taskParameter_.TaskEnable << "\n";
	}
	;

protected:
	TaskType type_;
	std::string ID_;
	double minBound_, maxBound_;
	Eigen::MatrixXd Ai_, J_;
	Eigen::VectorXd x_dot_;
	TaskParameter taskParameter_;

};
}

#endif
