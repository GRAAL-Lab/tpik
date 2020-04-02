#ifndef __EQUALITYTASK_H__
#define __EQUALITYTASK_H__

#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <iostream>

namespace tpik {

/**
 * @brief EqualityTask class, derived from the Abstract class tpik::Task
 * @details Implementation of the Equality tasks provided with an internal activation function equal to 1 and the TaskParameter struct
 * aimed to store the saturation value, the gain for the reference computation and a boolean stating whether the task is active. \n
 * The derived classes must implement the following pure virtual methods:
 *
 * * UpdateJacobian() where the user must update the variable J_ which stores the task jacobian;
 *
 * * UpdateReference() where the user must update the variable x_ which stores the task reference.
 *
 * * Update() public method used to update the task variables, hence the implementation of the previous pure virtual method must be called in order to update all the class variables.
 *
 */

class EqualityTask : public Task {

public:
    /**
	 * @brief Constructor of TaskEquality Class.
     * @details Initialization of the class variables:\n
	 * Jacobian (taskSpace x DoF)\n
	 * Internal Activation Function eye(taskSpace x taskSpace)\n
	 * Reference (taskSpace x 1)
     * @param[in] ID task ID
	 * @param[in] taskSpace
	 * @param[in] DoF
	 */
    EqualityTask(const std::string ID, int taskSpace, int DoF);
    /**
	 * @brief Default De-Constructor of TaskEquality Class
	 */
    virtual ~EqualityTask();
    /**
     * @brief Method setting the TaskEquality Parameter.
	 * @param[in] taskParameter tpik::TaskParameter struct.
	 */
    void SetTaskParameter(TaskParameter taskParameter);
    /**
     * @brief Method setting the gain Parameter.
     * @param[in] gain taskParameter.
     */
    void SetTaskParameter(double gain);
    /**
     * @brief  Method used to check the initialization, hence that all the task parameters have been initializated before updating the task.\n
     * Such meethod must be called in the Update() method before any other method.
     * @note An exception is thrown if the task parameter has not been initialized yet.
     */
    void CheckInitialization() throw(ExceptionWithHow);
    /**
     * @brief Method that returns the task Parameter
	 * @returns tpik::TaskParameter
	 */
    TaskParameter GetTaskParameter();
    /**
	 * @brief Overload of the cout operator.
	 */
    friend std::ostream& operator<<(std::ostream& os, EqualityTask const& equality)
    {
        return os << "\033[1;37m"
                  << "EQUALITY TASK: " << equality.ID_ << "\n"
                  << std::setprecision(4) << "Internal Activation Function \n"
                  << "\033[0m" << equality.Ai_ << "\n"
                  << "\033[1;37m"
                  << "Jacobian \n"
                  << "\033[0m" << equality.J_ << "\n"
                  << "\033[1;37m"
                  << "Reference \n"
                  << "\033[0m" << equality.x_dot_ << "\n"
                  << "\033[0m" << equality.taskParameter_ << "\n";
    }

protected:
    /**
     * @brief  Method used to saturate the reference, such method must be called in the Update() method after the UpdateReference method.
     */
    void SaturateReference();
    /**
     * @brief Method saturating reference component wise  i.e. saturating each element of the vector individually.
     */
    void SaturateReferenceComponentWise();
    /**
	 * @brief Method updating the internal activation function.
     * @details Implementation of the pure virtual method of the base class task.
	 * For the equality tasks the internal activation function is equal to identity since their always active.
     * For this reason such method is called in the class constructor and there is no need to call it again.
	 */
    void UpdateInternalActivationFunction() override;

    TaskParameter taskParameter_; //!< The tpik::TaskParameter.
    bool initializedTaskParameter_{ false }; //!< The boolean used to check whether the task parameter have been initialized.
};
}

#endif
