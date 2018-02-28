#ifndef __INEQUALITYTASK_H__
#define __INEQUALITYTASK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "TPIKDefines.h"
#include "Task.h"

namespace tpik {
/**
 * @brief InequalityTask class derived from the Abstract class Task.
 * Abstract base InequalityTask class to implement the Inequality tasks, the derived classes must implement the pure virtual methods UpdateActivationFunction,
 * UpdateJacobian and UpdateReference.
 * Such class implements the Set and Get minimum and maximum Bound in order to state the interval where the task is active.
 * Furthermore it is also possible to set and get the task parameter, composed by gain and a boolean stating whether the task is enabled
 * *  */
class InequalityTask: public Task {
public:
	/**
	 * @brief Constructor of Task Class.
	 * @param[in] ID: Task ID.
	 *  */
	InequalityTask(const std::string ID); // ID is set by the user in order to uniquely identify the task
	/**
	 * @brief Default De-constructor of InequalityTask Class.
	 *  */
	virtual ~InequalityTask();
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
	 * @brief Method that sets the Task Parameter (gain, bell shaped function parameter, Task enable boolean).
	 * @param[in] TaskParameter: Task Parameter struct.
	 *  */
	void SetTaskParameter(TaskParameter taskParameter);
	/**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns TaskParameter
	 */

	TaskParameter GetTaskParameter();
	/**
	 * @brief Method settings bell shaped parameter.
	 * @param[in] BellShapedParameter
	 */
	void SetBellShapedParameter(BellShapedParameter bellShapedParameter);
	/**
	 * @brief Method returning the task bell shaped parameter
	 * @returns bellShapedParameter
	 */
	BellShapedParameter GetBellShapedParameter();

protected:

	double minBound_, maxBound_;
	TaskParameter taskParameter_;
	BellShapedParameter bellShapedParameter_;

};
}

#endif
