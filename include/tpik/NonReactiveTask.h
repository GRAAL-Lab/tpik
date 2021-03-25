#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <iostream>

namespace tpik {
/*
 * @brief Non Reactive Task class, derived from the abstract class tpik::Task
 * @details Implementation of the Non Reactive tasks provided with an internal activation function equal to 1 and the TaskParameter struct
 * aimed to store the saturation value, the gain alway equal to 1 and a boolean stating whether the task is active.
 * The derived classes must implement the following pure virtual methods:
 * Update() public method used to update the task variables, hence the implementation of the previous virtual method must be called in order
 * to update all the class variables.
 */
class NonReactiveTask : public Task {

public:
    /*
    * @brief Constructor of NonReactiveTask Class.
    * @details Initialization of the class variables:
    * @param ID task ID
    * @param taskSpace
    * @param DoF
    */
    NonReactiveTask(const std::string ID, int taskSpace, int DoF);
    /*
    * @brief Default deconstructor.
    */
    ~NonReactiveTask() override;
    /*
     * @brief Method setting the task parameter.
     */
    auto TaskParameter() -> TaskParameter&
    {
        initializedTaskParameter_ = true;
        return taskParameter_;
    }
    /*
     * @brief Method getting and setting the task parameter.
     * @return Task parameter.
     */
    auto TaskParameter() const -> const struct TaskParameter& { return taskParameter_; }

    /*
    * @brief Method getting the control vector reference in case of equality tasks.
    * @return task reference x_dot_bar in body frame.
    */
    auto ReferenceRate() const -> const Eigen::VectorXd& { return x_dot_bar_; }
    /*
    * @brief Method to flag the possibility to saturete the reference component by component
    */
    auto SaturateReferenceRateComponentWise() -> bool&
    {
        saturateRaferenceRateComponentWise_ = true;
        return saturateRaferenceRateComponentWise_;
    }
    /*
    * @brief Method to config from file the task.
    */
    bool ConfigFromFile(libconfig::Config& confObj) noexcept(false) override;
    /*
    * @brief Overload of the cout operator.
    */
    friend std::ostream& operator<<(std::ostream& os, NonReactiveTask const& nonReactive)
    {
        return os << "\033[1;37m"
                  << static_cast<const Task&>(nonReactive)
                  << "TaskParameter\n"
                  << "\033[0m" << nonReactive.taskParameter_ << "\n"
                  << "Reference Rate\n"
                  << "\033[0m" << nonReactive.x_dot_bar_ << "\n";
    }

protected:
    /*
    * @brief  Method used to check the initialization, hence that all the task parameters have been initializated.
    * Such meethod must be called in the Update() method before any other method.
    * @note An exception is thrown if the task parameter has not been initialized yet.
    */
    virtual void CheckInitialization() noexcept(false);

    /*
    * @brief  Implementation of the pure virtual method of the base class Task used to update the internal activation function.
    * Such method must be called in the Update method. For non reactive tasks there is no need to use an activaction function.
    * For compliance with Reactive task is used and is set to the identity
    */
    void UpdateInternalActivationFunction() override;
    /*
    * @brief Implementation of the pure virtual method of the base class Task used to update the task reference rate.
    * Such method must be called in the Update method.
    */
    void UpdateReferenceRate() override;

    /*
    * @brief  Method used to saturate the reference, such method must be called in the Update() method after the UpdateReference method.
    */
    void SaturateReferenceRate();

    bool initializedTaskParameter_; // The boolean used to check whether the task parameter have been initialized.
    struct TaskParameter taskParameter_; // The tpik::TaskParameter.
    Eigen::VectorXd x_dot_bar_; // The control vector reference
    bool saturateRaferenceRateComponentWise_; //flag to check if the refarence rate must be saturete as vector or component by component
};
}
