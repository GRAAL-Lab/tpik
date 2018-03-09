#ifndef __EQUALITYTASK_H__
#define __EQUALITYTASK_H__

#include <iostream>
#include <vector>
#include <iostream>
#include <iomanip>
#include "TPIKDefines.h"
#include "Task.h"

namespace tpik {

/**
 * @brief EqualityTask class, derived from the Abstract class Task.
 * Abstract EqualityTask class, implementing the Equality tasks. The derived classes must implement the pure virtual methods UpdateActivationFunction,
 * UpdateJacobian and UpdateReference.
 * Furthermore the class is provided with method to set and get the task parameter, composed by gain and a boolean stating whether the task is enabled.
 */

class EqualityTask: public Task {

public:
	/**
	 * @brief Constructor of TaskEquality Class.
	 * @param[in] ID: Task ID.
	 */
	EqualityTask(const std::string ID, int TaskSpace, int DoF);
	/**
	 * @brief Default De-Constructor of TaskEquality Class
	 */
	virtual ~EqualityTask();
	/**
	 * @brief Method that sets the TaskEquality Parameter (gain, bell shaped function parameter, Task enable boolean).
	 * @param[in] TaskParameter: Task Parameter struct.
	 */
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
