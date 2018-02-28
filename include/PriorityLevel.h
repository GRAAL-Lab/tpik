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
/**
 * @brief PriorityLevel class
 * Implementation of the PriorityLevel Class. Starting form a vector of tpik::Task which have the same priority, it computes the
 * Jacobian, Internal Activation Function and Reference by juxtaposing the related task variables.
 * The External Activation Function (which depends on the current Action) is set, it is assumed that all the Tasks have the same
 * behavior (hence are all active or inactive).
 * The overall Activation Function is computed as product between the Internal and External activation functions.
 *  */
class PriorityLevel {
public:
	/**
	 * @brief Constructor of PriorityLevel Class.
	 * @param[in] ID: Priority Level ID.
	 *  */
	PriorityLevel(std::string ID);
	/**
	 * @brief Default Constructor of PriorityLevel Class.
	 *  */
	PriorityLevel();
	/**
	 * @brief Default De-constructor of PriorityLevel Class.
	 *  */
	~PriorityLevel();
	/**
	 * @brief Method which adds a tpik::Task to the PriorityLevel.
	 * @param[in] task: shared_ptr to the Task to be added.
	 *  */
	void AddTask(std::shared_ptr<Task> task);
	/**
	 * @brief Method returning the PriorityLevel ID.
	 * @return PriorityLevel ID.
	 *  */
	std::string GetID() const throw (PriorityLevelIndexException);
	/**
	 * @brief Method setting the PriorityLevel ID.
	 * @param[in] ID: priorityLevelID to be set.
	 *  */
	void SetID(std::string ID);
	/**
	 * @brief Method which juxtapose the Task Jacobians to obtain the priority level Jacobian.
	 *  */
	void UpdateJacobian();
	/**
	 * @brief Method which juxtapose the Task Internal Activation Functions to obtain the priority level Internal Activation Function.
	 *  */
	void UpdateInternalActivationFunction();
	/**
	 * @brief Method which juxtapose the Task References to obtain the priority level References.
	 *  */
	void UpdateReference();
	/**
	 * @brief Method which updates the PriorityLevel Jacobian, Internal ActivationFunction and Reference.
	 *  */
	void UpdateAll();
	/**
	 * @brief Method setting the external activation function of the PriorityLevel, it is supposed that such value is equal for all the priority Level tasks.
	 *
	 *  */
	void SetExternalActivationFunction(double Ae);
	/**
	 * @brief Method setting the PriorityLevel svdParameters.
	 * @param[in] svdParameters struct.
	 *  */
	void SetSVDParameters(rml::SVDParameters svdParameters);
	/**
	 * @brief Method Returning the PriorityLevel Jacobian.
	 * @return PriorityLevel Jacobian.
	 *  */
	const Eigen::MatrixXd& GetJacobian() const;
	/**
	 * @brief Method Returning the PriorityLevel Activation Function computed as follows : Ae*Ai.
	 * @return Activation Function.
	 *  */
	Eigen::MatrixXd GetActivationFunction();
	/**
	 * @brief Method returning the PriorityLevel Internal Activation Function.
	 * @return Internal Activation Function.
	 *  */
	const Eigen::MatrixXd& GetInternalActivationFunction() const;
	/**
	 * @brief Method returning the PriorityLevel External Activation Function.
	 * @return External Activation Function.
	 *  */
	double GetExternalActivationFunction();
	/**
	 * @brief Method returning the PriorityLevel Reference.
	 * @return PriorityLevel Reference.
	 *  */
	const Eigen::VectorXd& GetReference() const;
	/**
	 * @brief Method returning the PriorityLevel number of tpik::Task.
	 * @return Number of Task.
	 *  */
	int GetNumberOfTask();
	/**
	 * @brief Method returning the PriorityLevel Tasks as vector of shared_ptr to tpik::Task.
	 * @return Vector of shared_ptr of tpik::Task.
	 *  */
	const std::vector<std::shared_ptr<Task> > GetLevel() const;
	/**
	 * @brief Function Returning the PriorityLevel SVDParameters.
	 * @return PriorityLevel rml::SVDParameter.
	 *  */
	rml::SVDParameters GetSVDParameter();
	/**
	 * @brief Function overloading the cout operator
	 *  */
	friend std::ostream& operator <<(std::ostream& os, PriorityLevel const& priorityLevel) {
		return os << "\033[1;37m" << "PriorityLevel ID " << priorityLevel.ID_ << "\n" << std::setprecision(2)
				<< "\033[1;37m" << "Internal Activation Function \n" << "\033[0m" << priorityLevel.Ai_ << "\n"
				<< "\033[1;37m" << "External Activation Function " << "\033[0m" << priorityLevel.Ae_ << "\n"
				<< "\033[1;37m" << "Jacobian \n" << "\033[0m" << priorityLevel.J_ << "\n" << "\033[1;37m"
				<< "Reference \n" << "\033[0m" << priorityLevel.x_dot_ << "\n" << "\033[1;37m"
				<< "svdParameters\nThrehsold " << "\033[0m" << priorityLevel.svdParameters_.threshold << "\n"
				<< "\033[1;37m" << "lambda " << "\033[0m" << priorityLevel.svdParameters_.lambda << "\n" << "\033[1;37m"
				<< "mu " << "\033[0m" << priorityLevel.svdParameters_.mu << "\n" << "\033[0m";
	}


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
