#ifndef __EQUALITYTASK_H__
#define __EQUALITYTASK_H__

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
 * The derived class can implement either an Equality and Inequality Task by specifying it in the type attribute.
 * If the derived class implements an Inequality Task, also the minimum and/or the maximum bound must be set.
 *  */
class EqualityTask : public Task {

public:
	/**
	 * @brief Constructor of TaskEquality Class.
	 * @param[in] ID: Task ID.
	 *  */
	EqualityTask(const std::string ID); // ID is set by the user in order to uniquely identify the task
	/**
	 * @brief Default De-Constructor of TaskEquality Class
	 */
	virtual ~EqualityTask();
	/**
	 * @brief Method that sets the TaskEquality Parameter (gain, bell shaped function parameter, Task enable boolean).
	 * @param[in] TaskParameter: Task Parameter struct.
	 *  */
	void SetTaskParameter(TaskParameter taskParameter);

	/**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns TaskParameter
	 */

	TaskParameter GetTaskParameter();

protected:
	TaskParameter taskParameter_;

};
}

#endif
