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
 * @brief Task class.
 * Implementation of the abstract Task base class. The derived classes must implement the pure virtual methods: UpdateJacobian, UpdateInternalActivationFunction
 * and UpdateReference.
 */
class Task {
public:
	/**
	 * @brief Constructor of Task Class.
	 * @param[in] ID: Task ID.
	 */
	Task(const std::string ID, int TaskSpace, int DoF);
	/**
	 * @brief Default De-constructor of Task Class.
	 */
	virtual ~Task();
	/**
	 * @brief Method that sets the Task ID.
	 * @param[in] ID : Task ID.
	 */
	void SetID(const std::string ID);
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
     * @brief Pure Virtual Method to be implemented by the derived class to update the Task.
     */
	virtual void Update()=0;
	/**
	 * @brief Overload of the cout operator.
	 */
	friend std::ostream& operator <<(std::ostream& os, Task const& task) {
		return os << "\033[1;37m" << "Task ID " << task.ID_ << "\n" << std::setprecision(2) << "\033[1;37m"
				<< "Internal Activation Function \n" << "\033[0m" << task.Ai_ << "\n" << "\033[1;37m"
				<< "Jacobian \n" << "\033[0m" << task.J_ << "\n" << "\033[1;37m"
				<< "Reference \n" << "\033[0m" << task.x_dot_ << "\n";
	}

protected:
	/**
	 * @brief Pure Virtual Method to be implemented by the derived class to update the Task internal activation function.
	 */
	virtual void UpdateInternalActivationFunction()=0;
	/**
	 * @brief Pure Virtual Method to be implemented by the derived class to update the Task reference.
	 */
	virtual void UpdateReference()=0;
	/**
	 * @brief Pure Virtual Method to be implemented by the derived class to update the Task Jacobian.
	 *  */
	virtual void UpdateJacobian()=0;
	std::string ID_;
	Eigen::MatrixXd Ai_, J_;
	Eigen::VectorXd x_dot_;
	int taskSpace_;
	int DoF_;

};
}

#endif
