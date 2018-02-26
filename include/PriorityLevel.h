#ifndef __PRIORITYLEVEL_H__
#define __PRIORITYLEVEL_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>
#include <memory>
#include "Task.h"
#include "TPIKExceptions.h"

namespace tpik {
class PriorityLevel {
public:
	/**
	 * @brief Constructor of PriorityLevel Class
	 * @param[in] ID: Priority Level ID
	 *  */
	PriorityLevel(std::string ID);
	/**
	 * @brief Default Constructor of PriorityLevel Class
	 *  */
	PriorityLevel();
	/**
	 * @brief Default Deconstructor of PriorityLevel Class
	 *  */
	~PriorityLevel();
	/**
	 * @brief Add task to the priority Level
	 * @param[in] shared_ptr to the Task to be added
	 *  */
	void AddTask(std::shared_ptr<Task> task);
	/**
	 * @brief Function returning the priorityLevel ID
	 *  */
	/**
	 *
	 * @return
	 */
	std::string GetID() const throw (PriorityLevelIndexException);
	/**
	 * @brief Function setting the priorityLevel ID
	 * @param[in] ID: priorityLevelID to be set
	 *  */
	void SetID(std::string ID);
	/**
	 * @brief Function which juxtapose the Task Jacobians to obtain the priority level Jacobian
	 *  */
	void UpdateJacobian();
	/**
	 * @brief Function which juxtapose the Task Internal Activation Functions to obtain the priority level Internal Activation Function
	 *  */
	void UpdateInternalActivationFunction();
	/**
	 * @brief Function which juxtapose the Task References to obtain the priority level References
	 *  */
	void UpdateReference();
	/**
	 * @brief Function which updates the priorityLevel Jacobian, Internal ActivationFunction and Reference
	 *  */
	void UpdateAll();
	/**
	 * @brief Function setting the external activation function of the priorityLevel, it is supposed that such value is equal for all the priority Level tasks
	 *
	 *  */
	void SetExternalActivationFunction(double Ae);
	/**
	 * @brief Function setting the priorityLevel svdParameters
	 * @param[in] svdParameters struct
	 *  */
	void SetSVDParameters(rml::SVDParameters svdParameters);
	/**
	 * @brief Function Returning the priorityLevel Jacobian
	 *  */
	const Eigen::MatrixXd& GetJacobian() const;
	/**
	 * @brief Function Returning the priorityLevel Activation Function computed as follows : Ae*Ai
	 *  */
	Eigen::MatrixXd GetActivationFunction();
	/**
	 * @brief Function Returning the priorityLevel Internal Activation Function computed as follows : Ae*Ai
	 *  */
	const Eigen::MatrixXd& GetInternalActivationFunction() const;
	/**
	 * @brief Function Returning the priorityLevel External Activation Function.
	 *  */
	double GetExternalActivationFunction();
	/**
	 * @brief Function Returning the priorityLevel Reference.
	 *  */
	const Eigen::VectorXd& GetReference() const;
	/**
	 * @brief Function Returning the priorityLevel number of Tasks.
	 *  */
	int GetNumberOfTask();
	/**
	 * @brief Function Returning the priorityLevel Tasks as vector of shared_ptr to tpik::Task.
	 *  */
	const std::vector<std::shared_ptr<Task> > GetLevel() const;
	/**
	 * @brief Function Returning the priorityLevel SVDParameters.
	 *  */
	rml::SVDParameters GetSVDParameter();
	/**
	 * @brief Function overloading the cout operator
	 *  */
	friend std::ostream& operator <<(std::ostream& os,
			PriorityLevel const& priorityLevel) {
		return os << "\033[1;37m" << "PriorityLevel ID " << priorityLevel.ID_
				<< "\n" << std::setprecision(2) << "\033[1;37m"
				<< "Internal Activation Function \n" << "\033[0m"
				<< priorityLevel.Ai_ << "\n" << "\033[1;37m"
				<< "External Activation Function " << "\033[0m"
				<< priorityLevel.Ae_ << "\n" << "\033[1;37m" << "Jacobian \n"
				<< "\033[0m" << priorityLevel.J_ << "\n" << "\033[1;37m"
				<< "Reference \n" << "\033[0m" << priorityLevel.x_dot_ << "\n"
				<< "\033[1;37m" << "svdParameters\nThrehsold " << "\033[0m"
				<< priorityLevel.svdParameters_.threshold << "\n"
				<< "\033[1;37m" << "lambda " << "\033[0m"
				<< priorityLevel.svdParameters_.lambda << "\n" << "\033[1;37m"
				<< "mu " << "\033[0m" << priorityLevel.svdParameters_.mu << "\n"
				<< "\033[0m";
	}
	;

private:
	std::vector<std::shared_ptr<Task> > level_;
	std::string ID_;
	Eigen::MatrixXd Ai_, J_;
	Eigen::VectorXd x_dot_;
	double Ae_;
	int taskNumber_;
	rml::SVDParameters svdParameters_;
};
}
typedef std::vector<std::shared_ptr<tpik::PriorityLevel> > Hierarchy;

#endif
