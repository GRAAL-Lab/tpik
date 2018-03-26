#ifndef __EQUALITYTASK_H__
#define __EQUALITYTASK_H__

#include <iostream>
#include <iostream>
#include "TPIKDefines.h"
#include "Task.h"

namespace tpik
{

/**
 * @brief EqualityTask class, derived from the Abstract class Task.
 * Abstract EqualityTask class, implementing the Equality tasks. The derived classes must implement the pure virtual methods
 * UpdateJacobian and UpdateReference.
 * Furthermore the class is provided with method to set and get the task parameter, composed by gain, reference saturation value and
 * a boolean stating whether the task is enabled.
 */

class EqualityTask: public Task
{

public:
	/**
	 * @brief Constructor of TaskEquality Class.
	 * @param[in] ID: Task ID.
	 */
	EqualityTask(const std::string ID, int TaskSpace, int DoF);
	/**
	 * @brief Default De-Constructor of TaskEquality Class
	 */
	virtual ~EqualityTask();s
	/**
	 * @brief Method that sets the TaskEquality Parameter (gain, bell shaped function parameter, Task enable boolean).
	 * @param[in] TaskParameter: Task Parameter struct.
	 */
	void SetTaskParameter(TaskParameter taskParameter);
	/**
	 * @brief Method which check whether all the needed variables have been initialized.
	 * @note An exception is thrown if the Task parameter has not been initialized yet.
	 */
	void CheckInitialization() throw (std::exception);
	/**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns TaskParameter
	 */
	TaskParameter GetTaskParameter();
protected:
	/**
	 * @brief Method saturating the reference using the member variable saturation of the taskParameter struct.
	 */
	void SaturateReference();
	/**
	 * @brief Method updating the internal activation function.
	 * Implementation of the pure virtual method of the base class task.
	 * For the equality tasks the internal activation function is equal to identity since their always active.
	 * for this reason such method is called in the class constructor.
	 */
	void UpdateInternalActivationFunction() override;

	TaskParameter taskParameter_;
	bool initializedTaskParameter_;

};
}

#endif
