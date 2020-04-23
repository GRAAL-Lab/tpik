#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <iostream>

namespace tpik {

/**
 * @brief Non reactive Task class, derived from the abstract class tpik::Task
 * @details Implementation of the Non Reactive tasks provided with an internal activation function equal to 1 and the TaskParameter struct
 * aimed to store the saturation value, the gain alway equal to 1 and a boolean stating whether the task is active.
 * The derived classes must implement the following pure virtual methods:
 * Update() public method used to update the task variables, hence the implementation of the previous pure virtual method must
 * be called in order to update all the class variables.
 */
class NonReactiveTask : public Task {

public:
    /**
     * @brief Constructor of NonReactiveTask Class.
     * @details Initialization of the class variables:
     * Jacobian (taskSpace x DoF)
     * Internal Activation Function eye(taskSpace x taskSpace)
     * Reference (taskSpace x 1)
     * @param[in] ID task ID
     * @param[in] taskSpace
     * @param[in] DoF
     */
    NonReactiveTask(const std::string ID, int taskSpace, int DoF);
    /*
     *  @brief  ~ReactiveTask default deconstructor.
    */
    ~NonReactiveTask() override;
    /**
     * @brief  Method returning and setting the task parameter.
     * @return  task parameter.
     */
    auto TaskParameter() -> TaskParameter& { return taskParameter_; }
    auto TaskParameter() const -> const struct TaskParameter& { return taskParameter_; }
    /**
     * @brief  Method used to set the control vector reference in case of equality tasks.
     * @param xReference control vector reference.
     */
    auto Reference() -> Eigen::VectorXd&
    {
        isReferenceSet_ = true;
        return xReference_;
    }
    auto Reference() const -> const Eigen::VectorXd& { return xReference_; }
    /**
   * @brief Method to config from file the task
   */
    void ConfigFromFile(libconfig::Config& confObj) override;
    /**
         * @brief Overload of the cout operator.
         */
    friend std::ostream& operator<<(std::ostream& os, NonReactiveTask const& nonReactive)
    {
        return os << "\033[1;37m"
                  << dynamic_cast<const Task&>(nonReactive)
                  << "TaskParameter\n"
                  << "\033[0m" << nonReactive.taskParameter_ << "\n"
                  << "Reference\n"
                  << "\033[0m" << nonReactive.xReference_ << "\n";
    }

protected:
    /**
     * @brief  Method used to check the initialization, hence that all the task parameters have been initializated.
     * Such meethod must be called in the Update() method before any other method.
     * @note An exception is thrown if the task parameter has not been initialized yet.
     */
    void CheckInitialization() throw(ExceptionWithHow);
    /*
     * @brief  Implementation of the pure virtual method of the base class Task used to update the internal activation function.
       Such method must be called in the Update method. For non reactive tasks there is no need to use an activaction function.
       For compliance with Reactive task is used and is set to the identity
    */
    void UpdateInternalActivationFunction() override;
    /**
     * @brief Implementation of the pure virtual method of the base class Task used to update the task reference.
     * Such method must be called in the Update method.
     */
    void UpdateReference() override;
    /**
     * @brief  Method used to saturate the reference, such method must be called in the Update() method after the UpdateReference method.
     */
    void SaturateReference();
    /**
     * @brief Method saturating reference component wise  i.e. saturating each element of the vector individually.
     */
    void SaturateReferenceComponentWise();

    bool initializedTaskParameter_; // The boolean used to check whether the task parameter have been initialized.
    struct TaskParameter taskParameter_; // The tpik::TaskParameter.
    Eigen::VectorXd xReference_; // The control vector reference
    bool isReferenceSet_; //flag to check is the reference has been setted
};
}
