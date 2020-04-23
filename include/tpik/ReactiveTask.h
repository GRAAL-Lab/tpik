#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RML.h>

namespace tpik {

/**
 * @brief The ReactiveTask pure virtual class.
 * @details
 */
class ReactiveTask : public Task {
public:
    /**
     * @brief ReactiveTask class constructor.
     * @param ID task id
     * @param DoF degrees of freedom
     * @param taskType task type stating whether the class is equality, inequality greather then, inequality less the or inequality in between.
     */
    ReactiveTask(const std::string ID, int DoF, TaskType taskType);
    /*
     * @brief  ~ReactiveTask default deconstructor.
    */
    ~ReactiveTask() override;
    /**
     * @brief  Method returning and setting the task parameter.
     * @return  task parameter.
     */
    auto TaskParameter() -> TaskParameter& { return taskParameter_; }
    auto TaskParameter() const -> const struct TaskParameter& { return taskParameter_; }
    /**
    * @brief Method returning the task control variable.
    * @return Orientation error expressed wrt to the robot frame .
    */
    auto ControlVariable() const -> const Eigen::VectorXd& { return x_; }
    /**
    * @brief Method returning the task type.
    * @return task type.
    */
    auto Type() const -> const TaskType& { return taskType_; }
    /**
     * @brief  Method used to set the control vector reference in case of equality tasks.
     * @param xReference control vector reference.
     */
    auto Reference() -> Eigen::VectorXd& { return xReference_; }
    auto Reference() const -> const Eigen::VectorXd& { return xReference_; }
    /*
     * @brief Method to config from file the task
    */
    void ConfigFromFile(libconfig::Config& confObj) override;

    //    /**
    //     * @brief Overload of the cout operator.
    //     */
    //    friend std::ostream& operator<<(std::ostream& os, ReactiveTask const& ReactiveTask)
    //    {
    //        os << "\033[1;37m"
    //           << "CARTESIAN TASK : " << ReactiveTask.ID_ << "\n"
    //           << std::setprecision(4) << "Internal Activation Function \n"
    //           << "\033[0m" << ReactiveTask.Ai_ << "\n"
    //           << "\033[1;37m"
    //           << "Jacobian \n"
    //           << "\033[0m" << ReactiveTask.J_ << "\n"
    //           << "\033[1;37m"
    //           << "Reference \n"
    //           << "\033[0m" << ReactiveTask.x_dot_ << "\n"
    //           << "\033[1;37m";
    //        if (ReactiveTask.useErrorNorm_) {
    //            os << "x_ \n"
    //               << "\033[0m" << ReactiveTask.x_.norm() << "\n";
    //        } else {
    //            os << "x_ \n"
    //               << "\033[0m" << ReactiveTask.x_ << "\n";
    //        }
    //        //
    //        os << "\033[0m" << ReactiveTask.taskParameter_ << "\n"
    //           << "\033[1;37m"
    //           << "Use Error Norm  \n"
    //           << "\033[0m" << ReactiveTask.useErrorNorm_ << "\n";
    //        return os;
    //    }

protected:
    /**
     * @brief  Implementation of the pure virtual method of the base class Task used to update the internal activation
     * function.
     * Such method must be called in the Update method.
     */
    void UpdateInternalActivationFunction() override;
    /**
     * @brief Implementation of the pure virtual method of the base class Task used to update the task reference.
     * Such method must be called in the Update method.
     */
    void UpdateReference() override;
    /**
     * @brief  Method used to saturate the reference, such method must be called in the Update() method after the
     * UpdateReference method.
     */
    void SaturateReference();
    /**
     * @brief Method saturating reference component wise  i.e. saturating each element of the vector individually.
     */
    void SaturateReferenceComponentWise();
    /**
     * @brief  Method used to check the initialization, hence that all the task parameters have been initializated
     * before updating the task.
     * Such meethod must be called in the Update() method before any other method.
     * @note An exception is thrown if the task parameter has not been initialized yet.
     */
    void CheckInitialization() throw(ExceptionWithHow);

    Eigen::VectorXd x_; // The control vector
    Eigen::VectorXd xReference_; // The control vector reference
    struct TaskParameter taskParameter_; // The tpik::TaskParameter.
    BellShapedParameter increasingBellShapeParameter_; // The bell shape struct
    BellShapedParameter decreasingBellShapeParameter_; // The bell shape struct when the task type is inequality in between
    TaskType taskType_; // Enum stating whether the task type is either inequality greather then, inequality less then, inequality in beteeen or equality
    bool initializedTaskParameter_; // Boolean stating whether the task parameter have been initialized
    bool initializedBellShapeParameter_; // Boolean stating whether the bell shaped parameters have been initialized
};
}
