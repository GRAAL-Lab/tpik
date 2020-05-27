#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RML.h>

namespace tpik {

/*
* @brief The ReactiveTask pure virtual class.
* @details
*/
class ReactiveTask : public Task {
public:
    /*
    * @brief ReactiveTask class constructor.
    * @param ID - the task ID
    * @param taskSpace - the task space
    * @param dof - degrees of freedom
    * @param taskOption - Enum class to handle the different task option modalities:
    * - 0: Default,
    * - 1: UseErrorNorm,
    * - 2: ActiveOnNorm
    */
    ReactiveTask(const std::string ID, int taskSpace, int DoF, tpik::TaskOption taskOption);
    /*
    * @brief  ~ReactiveTask default deconstructor.
    */
    ~ReactiveTask() override;
    /*
    * @brief  Method returning and setting the task parameter.
    * @return  task parameter.
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
    * @brief Method returning the task control variable.
    */
    auto ControlVariable() const -> const Eigen::VectorXd& { return x_; }
    /*
    * @brief Method setting the task type.
    */
    auto Type() -> TaskType&
    {
        isTaskTypeSet_ = true;
        return taskType_;
    }
    /*
    * @brief Method getting the task type.
    * @return task type.
    */
    auto Type() const -> const TaskType& { return taskType_; }
    /*
    * @brief Method setting the greaternThan params.
    */
    auto GreaterThanParams() -> tpik::BellShapedParameter&
    {
        isGreaterThanParamsInizialized_ = true;
        return decreasingBellShapeParameter_;
    }
    /*
    * @brief Method getting the greaternThan params.
    */
    auto GreaterThanParams() const -> const BellShapedParameter& { return decreasingBellShapeParameter_; }
    /*
    * @brief Method setting the lessThan params
    */
    auto LessThanParams() -> tpik::BellShapedParameter&
    {
        isLessThanParamsInizialized_ = true;
        return increasingBellShapeParameter_;
    }
    /*
    * @brief Method getting the lessThan params
    */
    auto LessThanParams() const -> const BellShapedParameter& { return increasingBellShapeParameter_; }
    /*
    * @brief Method used setting the control vector reference.
    */
    auto Reference() -> Eigen::VectorXd& { return x_bar_; }
    /*
    * @brief Method getting control vector reference.
    */
    auto Reference() const -> const Eigen::VectorXd& { return x_bar_; }
    /*
    * @brief Method to config from file the task
    */
    void ConfigFromFile(libconfig::Config& confObj) override;
    /*
    * @brief Added the saturation on the reference rate on  the update
    */
    void Update() override;
    /*
    * @brief Overload of the cout operator.
    */
    friend std::ostream& operator<<(std::ostream& os, ReactiveTask const& reactiveTask)
    {
        os << "\033[1;37m"
           << static_cast<const Task&>(reactiveTask) << "\n";
        if (reactiveTask.taskOption_ == tpik::TaskOption::ActiveOnNorm || reactiveTask.taskOption_ == tpik::TaskOption::UseErrorNorm) {
            os << "x_ \n"
               << "\033[0m" << reactiveTask.x_.norm() << "\n";
        } else {
            os << "x_ \n"
               << "\033[0m" << reactiveTask.x_ << "\n";
        }
        os << "\033[0m" << reactiveTask.taskParameter_ << "\n";
        if (reactiveTask.taskType_ == tpik::TaskType::Inequality && reactiveTask.isLessThanParamsInizialized_) {
            os << "Less Then Params \n"
               << "\033[0m" << reactiveTask.increasingBellShapeParameter_ << "\n";
        } else if (reactiveTask.taskType_ == tpik::TaskType::Inequality && reactiveTask.isGreaterThanParamsInizialized_) {
            os << "Greater Than Params\n"
               << "\033[0m" << reactiveTask.decreasingBellShapeParameter_ << "\n";
        }

        return os;
    }

protected:
    /*
    * @brief  Implementation of the pure virtual method of the base class Task used to update the internal activation function.
    * Such method must be called in the Update method.
    */
    void UpdateInternalActivationFunction() override;
    /*
    * @brief Implementation of the pure virtual method of the base class Task used to update the task reference rate.
    * Such method must be called in the Update method.
    */
    void UpdateReferenceRate() override;
    /*
    * @brief Method updating the reference for reactive task.
    */
    void UpdateReference() override;
    /*
    * @brief Method updating the Jacobian.
    * Implementation of the pure virtual method of the base class tpik::Task.
    */
    void UpdateJacobian() override;
    /*
    * @brief  Method used to saturate the reference, such method must be called in the Update() method after the
    * UpdateReference method.
    */
    void SaturateReferenceRate();
    /*
    * @brief  Method used to check the initialization, hence that all the task parameters have been initializated before updating the task.
    * Such meethod must be called in the Update() method before any other method.
    * @note An exception is thrown if the task parameter has not been initialized yet.
    */
    virtual void CheckInitialization() noexcept(false);

    Eigen::VectorXd x_; // The control vector
    Eigen::VectorXd x_bar_; // The control vector reference
    struct TaskParameter taskParameter_; // The tpik::TaskParameter.
    BellShapedParameter increasingBellShapeParameter_; // The bell shape struct
    BellShapedParameter decreasingBellShapeParameter_; // The bell shape struct when the task type is inequality in between
    TaskType taskType_; // Enum stating whether the task type is either inequality greather then, inequality less then, inequality in beteeen or equality
    bool initializedTaskParameter_; // Boolean stating whether the task parameter have been initialized
    bool isLessThanParamsInizialized_, isGreaterThanParamsInizialized_; // Boolean stating whether the bell shaped parameters have been initialized
    bool isTaskTypeSet_; //  Boolean stating whether the task type has been set
    TaskOption taskOption_; // use ActivateOnNorm or ErrorNorm
    bool saturareRateComponentWise_; // flag to check if the refarence rate must be saturete as vector or component by component
    Eigen::MatrixXd AgreaterThan_, AlessThan_; // matrix containing the Activation value for the less than and greater than control objetive
};
}
