#ifndef __INEQUALITYTASK_H__
#define __INEQUALITYTASK_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "TPIKDefines.h"
#include "Task.h"

namespace tpik
{
/**
 * @brief InequalityTask class, derived from the Abstract class Task.
 * Abstract base InequalityTask, implementing the inequality task. The derived classes must implement the pure virtual methods UpdateActivationFunction,
 * UpdateJacobian and UpdateReference.
 * Such class is provided with method to set and get the minimum and maximum bounds in order to state the interval where the task is active.
 * Furthermore methods to set and get the task parameter (gain, reference saturation and a boolean stating whether the task is enabled) are implemented.
 */
class InequalityTask: public Task
{
public:
	/**
	 * @brief Constructor of Task Class.
	 * @param[in] ID: Task ID.
	 */
	InequalityTask(const std::string ID, int TaskSpace, int DoF);
	/**
	 * @brief Default De-constructor of InequalityTask Class.
	 */
	virtual ~InequalityTask();
	/**
	 * @brief Method that sets the Task Minimum bound for Inequality Task to define the interval in which the Task must be active.
	 * @param[in] minBound: eigen vector minimum interval value for each scalar task .
	 */
	void SetMinBound(Eigen::VectorXd minBound);

	/**
	 * @brief Method that sets the Task Maximum bound for Inequality Task to define the interval in which the Task must be active.
	 * @param[in] maxBound: eigen vector maximum interval value for each scalar task .
	 */
	void SetMaxBound(Eigen::VectorXd maxBound);

	/**
	 * @brief Method that sets the Task Parameter (gain, task enable boolean).
	 * @param[in] TaskParameters: Task Parameter struct.
	 */
	void SetTaskParameter(TaskParameter taskParameters);
	/**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns TaskParameter
	 */

	const TaskParameter& GetTaskParameter();
	/**
	 * @brief Method increasing bell shaped parameters.
	 * @param[in] increasingBellShapedParameters increasing bell shaped parameter.
	 */
	void SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShapedParameters);
	/**
	 * @brief Method settings decreasing bell shaped parameter.
	 * @param[in] decreasingBellShapedParameters decreasing bell shaped parameter.
	 */
	void SetDecreasingBellShapedParameter(BellShapedParameter decreasingBellShapedParameters);
	/**
	 * @brief Method returning the task increasing bell shaped parameter.
	 * @returns increasing bellShapedParameter.
	 */
	const BellShapedParameter& GetIncreasingBellShapedParameter();
	/**
	 * @brief Method returning the task decreasing bell shaped parameter.
	 * @returns increasing bellShapedParameter.
	 */
	const BellShapedParameter& GetDecreasingBellShapedParameter();
	/**
	 * @brief Method which chek whether all the needed variables have been initialized.
	 * @note An exception is thrown if either the bellShapedParameter or the Task parameter have not been initialized yet.
	 */
	void CheckInitialization() throw (std::exception);

protected:
	/**
	 * @brief Method saturating the reference using the member variable saturation of the taskParameter struct.
	 */
	void SaturateReference();
	Eigen::VectorXd minBound_, maxBound_;
	TaskParameter taskParameter_;
	bool initializedTaskParameter_,initializedDecreasingBellShapeParameter_, initializedIncreasingBellShapeParameter_;
	bool bellShapeIncreasingUsed_,bellShapeDecreasingUsed_;
	BellShapedParameter increasingBellShape_;
	BellShapedParameter decreasingBellShape_;

};
}

#endif
