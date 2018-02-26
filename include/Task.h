#ifndef __TASK_H__
#define __TASK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "TPIKDefines.h"

namespace tpik {
/**
 * @brief Task class
 * Implementation of the Task base class. The derived classes must implement the pure virtual methods: UpdateJacobian, UpdateInternalActivationFunction
 * and UpdateReference.
 * The derived class can implement either an Equality and Inequality Task by specifing it in the type attribute.
 * If the derived class implements an Inequality Task, also the minimum and/or the maximum bound must be set.
 *  */
class Task {
public:
	/**
	 * @brief Constructor of Task Class.
	 * @param[in] type: Task type: {Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween};
	 * @param[in] ID: Task ID.
	 *  */
	Task(TaskType type, const std::string ID); // ID is set by the user in order to uniquely identify the task
	/**
	 * @brief Constructor of Task Class.
	 * @param[in] type: Task type: {Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween}.
	 *  */
	Task(TaskType type);
	/**
	 * @brief Default De-constructor of Task Class.
	 *  */
	virtual ~Task();
	/**
	 * @brief Method that sets the Task ID.
	 * @param[in] ID : Task ID.
	 *  */
	void SetID(const std::string ID);
	/**
	 * @brief Method that sets the Task Minimum bound for Inequality Task to define the interval in which the Task must be active.
	 * @param[in] minBound: minimum interval value.
	 *  */
	void SetMinBound(double minBound);

	/**
	 * @brief Method that sets the Task Maximum bound for Inequality Task to define the interval in which the Task must be active.
	 * @param[in] maxBound: maximum interval value.
	 *  */
	void SetMaxBound(double maxBound);
	/**
	 * @brief Method returning the Jacobian Matrix of the Task.
	 * @return Jacobian Matrix.
	 *  */
	const Eigen::MatrixXd& GetJacobian() const;
	/**
	 * @brief Method returning the Internal Activation Function Matrix of the Task.
	 * @return Internal Activation Function.
	 *  */
	const Eigen::MatrixXd& GetInternalActivationFunction() const;
	/**
	 * @brief Method returning the Task Reference.
	 * @return Task Reference.
	 *  */
	const Eigen::VectorXd& GetReference() const;
	/**
	 * @brief Method that sets the Task Parameter (gain, bell shaped function parameter, Task enable boolean).
	 * @param[in] TaskParameter: Task Parameter struct.
	 *  */
	void SetTaskParameter(TaskParameter taskParameter);
	/**
	 * @brief Pure Virtual Method to be implemented by the derived class to update the Task internal activation function.
	 *  */
	virtual void UpdateInternalActivationFunction()=0;
	/**
	 * @brief Pure Virtual Method to be implemented by the derived class to update the Task reference.
	 *  */
	virtual void UpdateReference()=0;
	/**
	 * @brief Pure Virtual Method to be implemented by the derived class to update the Task Jacobian.
	 *  */
	virtual void UpdateJacobian()=0;
	/**
	 * @brief Overload of the cout operator.
	 *  */
	friend std::ostream& operator <<(std::ostream& os, Task const& task) {
		return os << "\033[1;37m" << "Task ID " << task.ID_ << "\n" << std::setprecision(2) << "\033[1;37m"
				<< "Internal Activation Function \n" << "\033[0m" << task.Ai_ << "\n" << "\033[1;37m"
				<< "Jacobian \n" << "\033[0m" << task.J_ << "\n" << "\033[1;37m"
				<< "Reference \n" << "\033[0m" << task.x_dot_ << "\n" << "\033[1;37m"
				<< "minBound \n" << "\033[0m" << task.minBound_ << "\n" << "\033[1;37m"
				<< "maxBound \n" << "\033[0m" << task.maxBound_ << "\n" << "\033[1;37m"
				<< "TaskParameter (Gain TaskEnable) \n"
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
