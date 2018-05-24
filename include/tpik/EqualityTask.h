#ifndef __EQUALITYTASK_H__
#define __EQUALITYTASK_H__

#include <iostream>
#include <iostream>
#include "TPIKDefines.h"
#include "Task.h"

namespace tpik
{

/**
 * @brief EqualityTask class, derived from the Abstract class tpik::Task.\n
 * Implementation of the Equality tasks. The derived classes must implement the pure virtual methods
 * UpdateJacobian and UpdateReference.\n
 * Furthermore the class is provided with methods to set and get the tpik::TaskParameter, composed by gain, reference saturation value and
 * a boolean stating whether the task is enabled.
 */

class EqualityTask: public Task
{

public:
	/**
	 * @brief Constructor of TaskEquality Class.
	 * Initialization of the class variables:\n
	 * Jacobian (taskSpace x DoF)\n
	 * Internal Activation Function eye(taskSpace x taskSpace)\n
	 * Reference (taskSpace x 1)
	 * @param[in] ID Task ID.
	 * @param[in] taskSpace
	 * @param[in] DoF
	 */
	EqualityTask(const std::string ID, int taskSpace, int DoF);
	/**
	 * @brief Default De-Constructor of TaskEquality Class
	 */
	virtual ~EqualityTask();
	/**
	 * @brief Method that sets the TaskEquality Parameter (gain, bell shaped function parameter, Task enable boolean).
	 * @param[in] taskParameter tpik::TaskParameter struct.
	 */
	void SetTaskParameter(TaskParameter taskParameter);
	/**
	 * @brief Method which check whether all the needed variables have been initialized.
	 * @note An exception is thrown if the Task parameter has not been initialized yet.
	 */
	void CheckInitialization() throw (std::exception);
	/**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns tpik::TaskParameter
	 */
	TaskParameter GetTaskParameter();
	/**
	 * @brief Overload of the cout operator.
	 */
	friend std::ostream& operator <<(std::ostream& os, EqualityTask const& equality)
	{
		return os <<"\033[1;37m"<<"EQUALITY TASK: " << equality.ID_<<"\n" <<std::setprecision(4)<< "Internal Activation Function \n" << "\033[0m" << equality.Ai_ << "\n"
				<< "\033[1;37m" << "Jacobian \n" << "\033[0m" << equality.J_ << "\n"
				<< "\033[1;37m" << "Reference \n" << "\033[0m" << equality.x_dot_ <<"\n"
				<< "\033[0m" << equality.taskParameter_ <<"\n";
	}
protected:
	/**
	 * @brief Method saturating the reference using the member variable saturation of the tpik::TaskParameter struct.
	 */
	void SaturateReference();
	/**
	 * @brief Method updating the internal activation function.
	 * Implementation of the pure virtual method of the base class task.
	 * For the equality tasks the internal activation function is equal to identity since their always active.
	 * for this reason such method is called in the class constructor.
	 */
	void UpdateInternalActivationFunction() override;

	TaskParameter taskParameter_; //!< The tpik::TaskParameter.
	bool initializedTaskParameter_{false}; //!< The boolean used to check whether the task parameter have been initialized.

};
}

#endif
