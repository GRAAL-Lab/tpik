#include "TPIKExceptions.h"
#include "Task.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RML.h>

namespace tpik {
/**
 * @brief The TaskType enum to state whether the task is equality or inequality
 */
enum  class TaskType { Equality,
    Inequality };
/**
 * @brief The CartesianTask class .
 * @details The cartesian task is aimed to implement the basic operation for the cartesian task, as the change of observer and the possibility to implement
 * either the three dimensional task or the one dimensional task. i.e. project the jacobian on the error direction. \n
 * The tasks aimed to implement a cartesian control(as position, velocity, obstacle avoidance) can derived from the cartesianTask class and implement the pure virtual methods.
 */
class CartesianTask : public Task {
public:
    /**
     * @brief CartesianTask class constructor
     * @param ID task id
     * @param DoF degrees of freedom
     * @param taskType task type stating whether the class is equality or inequality
     */
    CartesianTask(const std::string ID, int DoF, TaskType taskType);
    /* @brief  ~CartesianTask default deconstructor
    */
    ~CartesianTask();
    /**
     * @brief SetTaskParameter Method to set the task parameters
     * @param taskParameters
     */
    void SetTaskParameter(TaskParameter taskParameters);
    /**
     * @brief GetTaskParameter Method returning the task parameter
     * @return  task parameter
     */
    const TaskParameter& GetTaskParameter();
    /**
     * @brief SetIncreasingBellShapedParameter Method used to set the increasing bell shape parameter if the task is of type inequality
     * @param increasingBellShapedParameters
     */
    void SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShapedParameters);
    /**
     * @brief GetIncreasingBellShapedParameter method returning the bell shape parameter
     * @return  increasing bell shape parameter
     */
    const BellShapedParameter& GetIncreasingBellShapedParameter();

    /**
     * @brief SetUseErrorNorm Method used in order to make the task one dimensional, i.e. project the jacobian along the error direction
     */
    void SetUseErrorNorm();
    /**
     * @brief Overload of the cout operator.
     */
    friend std::ostream& operator<<(std::ostream& os, CartesianTask const& cartesianTask)
    {
        os << "\033[1;37m"
           << "CARTESIAN TASK : " << cartesianTask.ID_ << "\n"
           << std::setprecision(4)
           << "Internal Activation Function \n"
           << "\033[0m" << cartesianTask.Ai_ << "\n"
           << "\033[1;37m"
           << "Jacobian \n"
           << "\033[0m" << cartesianTask.J_ << "\n"
           << "\033[1;37m"
           << "Reference \n"
           << "\033[0m" << cartesianTask.x_dot_ << "\n"
           << "\033[1;37m"
           << "Error \n"
           << "\033[0m" << cartesianTask.error_ << "\n"
           << "\033[0m" << cartesianTask.taskParameter_<<"\n"
           << "\033[1;37m"
           << "Use Error Norm  \n"
           << "\033[0m" << cartesianTask.useErrorNorm_ << "\n";

        if (cartesianTask.taskType_ == TaskType::Inequality) {
            os << "\033[1;37m"
               << "increasing bell shape\n"
               << "\033[0m" << cartesianTask.increasingBellShape_ << "\n";
        }
        return os;
    }

protected:
    /**
     * @brief ChangeObserver Method used to change the observer
     */
    void ChangeObserver();
    /**
     * @brief UpdateInternalActivationFunction Implementation of the pure virtual method of the base class Task used to update the internal activation function.
     * Such method must be called in the update function
     */
    void UpdateInternalActivationFunction() override;
    /**
     * @brief UpdateReference Implementation of the pure virtual method of the base class Task used to update the task reference.
     * Such method must be called in the update function
     */
    void UpdateReference() override;
    /**
     * @brief SaturateReference Method used to saturate the reference, such method must be called in the update function after the update reference method
     */
    void SaturateReference();
    /**
     * @brief CheckInitialization Method used to check the initialization hence that all the task parameters have been initializated before update the task.\n
     * Such meethod must be called in the upate function.
     */
    void CheckInitialization() throw(ExceptionWithHow);

    Eigen::Vector3d error_;//!< The error vector
    Eigen::MatrixXd JObserver_;//!< The observer jacobian wrt to inertial frame
    BellShapedParameter increasingBellShape_;//!< The increasing bell shape struct
    TaskParameter taskParameter_; //!< The task parameter struct
    bool useErrorNorm_{ false };//!< Boolean stating whether project the jacobian along the error direction
    bool initializedTaskParameter_{ false };//!< Boolean stating whether the task parameter have been initialized
    bool initializedIncreasingBellShapeParameter_{ false };//!< Boolean stating whether the increasing bell shaped parameters have been initialized
    TaskType taskType_;//!< Enum stating whether the task type is either inequality or equality
};
}
