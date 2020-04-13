#ifndef __INEQUALITYTASK_H__
#define __INEQUALITYTASK_H__

#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace tpik {

/**
 * @brief InequalityTask class, derived from the Abstract class Task \n
 * @details Implementation of the inequality task, such class is provided with methods to set and get the increasing bell shaped parameter and the decreasing bell shaped parameter in order to state the interval
 * where the task is active. \n
 * The derived classes must implement the following pure virtual methods:
 *
 * * UpdateJacobian() where the user must update the variable J_ which stores the task jacobian;
 *
 * * UpdateInternalActivationFunction() where the user must update the class variabel Ai_ which stores the task activation function. The user can decide to implement either an increasing, decreasing or in between bell shape activation function by using the bell shape parameter struct;
 *
 * * UpdateReference() where the user must update the variable x_ which stores the task reference;
 *
 * * Update() public method used to update the task variables, hence the implementation of the previous pure virtual method must be called in order to update all the class variables.
 *
 * Furthermore methods to set and get the task parameter (gain, reference saturation and a boolean stating whether the task is enabled) are implemented.
 */
class InequalityTask : public Task {
public:
    /**
     * @brief Constructor of Task Class.
     * @details Initialization of the class variables:\n
	 * Jacobian (taskSpace x DoF)\n
	 * Internal Activation Function (taskSpace x taskSpace)\n
	 * Reference (taskSpace x 1)\n
	 * @param[in] ID Task ID.
	 * @param[in] taskSpace;
	 * @param[in] DoF;
     * @param[in] Inequality type enum stating whether the task is an increasing, decreasing, inbetween or none of the above inequality task.
     */
    InequalityTask(const std::string ID, int taskSpace, int DoF, InequalityTaskType inequalityType);
    /**
	 * @brief Default De-constructor of InequalityTask Class.
	 */
    virtual ~InequalityTask();
    /**
	 * @brief Method that sets the Task Parameter (gain, task enable boolean).
	 * @param[in] taskParameters tpik::TaskParameter struct.
	 */
    void SetTaskParameter(TaskParameter taskParameters) override;

    /**
     * @brief Method setting the gain Parameter.
     * @param[in] gain taskParameter.
     */
    void SetTaskParameter(double gain) override;

    /**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns TaskParameter
	 */
    const TaskParameter& GetTaskParameter();
    /**
	 * @brief Method increasing bell shaped parameters.
	 * @param[in] increasingBellShapedParameters tpik::BellShapedParameter for increasing curve.
	 */
    void SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShapedParameters);
    /**
	 * @brief Method settings decreasing bell shaped parameter.
	 * @param[in] decreasingBellShapedParameters tpik::BellShapedParameter for decreasing curve.
	 */
    void SetDecreasingBellShapedParameter(BellShapedParameter decreasingBellShapedParameters);
    /**
	 * @brief Method returning the task increasing bell shaped parameter.
	 * @returns tpik::BellShapedParameter for increasing curve.
	 */
    const BellShapedParameter& GetIncreasingBellShapedParameter();
    /**
	 * @brief Method returning the task decreasing bell shaped parameter.
	 * @returns tpik::BellShapedParameter for decreasing curve.
	 */
    const BellShapedParameter& GetDecreasingBellShapedParameter();
    /**
     * @brief  Method used to check the initialization, hence that all the task parameters have been initializated before updating the task.\n
     * Such meethod must be called in the Update() method before any other method.
     * @note An exception is thrown if the task parameter has not been initialized yet.
     */
    void CheckInitialization() throw(ExceptionWithHow);
    /**
	 * Method stating whether the increasing bell shape parameter are used.
	 * @return true if the bell shape increasing parameter are used, false otherwise.
	 */
    bool GetBellShapeIncreasingUsed();
    /**
	 * Method stating whether the decreasing bell shape parameter are used.
	 * @return true if the bell shape decreasing parameter are used, false otherwise.
	 */
    bool GetBellShapeDecreasingUsed();

    InequalityTaskType GetType();

    TaskType GetTaskType() override;

    /**
	 * @brief Overload of the cout operator.
	 */
    friend std::ostream& operator<<(std::ostream& os, InequalityTask const& inequality)
    {
        os << "\033[1;37m"
           << "INEQUALITY TASK : " << inequality.ID_ << "\n"
           << std::setprecision(4)
           << "Internal Activation Function \n"
           << "\033[0m" << inequality.Ai_ << "\n"
           << "\033[1;37m"
           << "Jacobian \n"
           << "\033[0m" << inequality.J_ << "\n"
           << "\033[1;37m"
           << "Reference \n"
           << "\033[0m" << inequality.x_dot_ << "\n"
           << "\033[0m" << inequality.taskParameter_;

        if (inequality.inequalityType_ == InequalityTaskType::Decreasing || inequality.inequalityType_ == InequalityTaskType::Inbetween) {
            os << "\033[1;37m"
               << "decreasing bell shape\n"
               << "\033[0m" << inequality.decreasingBellShape_ << "\n";
        }
        if (inequality.inequalityType_ == InequalityTaskType::Increasing || inequality.inequalityType_ == InequalityTaskType::Inbetween) {
            os << "\033[1;37m"
               << "increasing bell shape\n"
               << "\033[0m" << inequality.increasingBellShape_ << "\n";
        }
        return os;
    }

protected:
    /**
     * @brief  Method used to saturate the reference, such method must be called in the Update() method after the UpdateReference() method.
     */
    void SaturateReference();
    /**
     * @brief Method saturating reference component wise  i.e. saturating each element of the vector individually.
     */
    void SaturateReferenceComponentWise();

    bool initializedTaskParameter_{ false }; //!< The boolean stating whether the tpik::TaskParameter struct has been initialized
    bool initializedDecreasingBellShapeParameter_{ false }; //!< The boolean stating whether the tpik::BellShapedParameter struct for increasing curve has been initialized
    bool initializedIncreasingBellShapeParameter_{ false }; //!< The boolean stating whether the tpik::BellShapedParameter struct for decreasing curve has been initialized
    InequalityTaskType inequalityType_; //!< variable stating the kind of inequality task
    BellShapedParameter increasingBellShape_; //!< The tpik::BellShapedParameter for increasing curve
    BellShapedParameter decreasingBellShape_; //!< The tpik::BellShapedParameter for decreasing curve
};
}

#endif
