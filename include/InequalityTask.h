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
 * @brief Task class
 * Implementation of the Task base class. The derived classes must implement the pure virtual methods: UpdateJacobian, UpdateInternalActivationFunction
 * and UpdateReference.
 * The derived class can implement either an Equality and Inequality Task by specifing it in the type attribute.
 * If the derived class implements an Inequality Task, also the minimum and/or the maximum bound must be set.
 *  */
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
