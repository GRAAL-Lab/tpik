#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include "Task.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RML.h>

namespace tpik {

/**
 * @brief The CartesianTask pure virtual class.
 * @details The class is aimed to implement the basic common methods needed by a cartesian task. \n
 * It allows to decide whether implement the three  or the one dimensional task when instantiating the obect.  \n
 * The task is by default a three dimensional task, in order to have a one dimensional task, hence projecting the jacobian along the error direction, one must call the method SetOneDimensional().\n
 * When instantiating the object the user can decide to create one of the following task:\n
 * * Equality task: The task tries to drive the control variabel towards the reference value. If no reference value is setted via the dedicated method SetControlVectorReference() , the default value 0 is used;
 *
 * * Inequality increasing task: The task tries to decrease the control variable towards the reference value. The reference value is stored in the member xmin of the bellShapeParameter struct that must be
 * setted via the dedicated method SetBellShapedParameter(). The name "Inequality Increasing" derives from the fact that the task activation function is an increasing bell shape function wrt the control variable;
 *
 * * Inequality Decreasing: The task tries to increae the control variable towards the reference value. The reference value is stored in the member xmax of the bellShapeParameter struct that must be
 * setted via the dedicated method SetBellShapedParameter(). The name "Ineqaulity Decreasing" derives from the fact that the task activation function is a decreasing bell shape function wrt the control variable;
 *
 * When instantiating the object, the user must also set the taskParameter by using the dedicated method SetTaskParameter().\n
 * The tasks aimed to implement a cartesian control (e.g. position, velocity... ) can derive from the cartesianTask class and implement the pure virtual methods Update() and UpdateJacobian().
 */
class CartesianTask : public Task {
public:
    /**
     * @brief CartesianTask class constructor.
     * @param ID task id
     * @param DoF degrees of freedom
     * @param taskType task type stating whether the class is equality, inequality increasing, inequality decreasing.
     */
    CartesianTask(const std::string ID, int DoF, CartesianTaskType taskType);

    /* @brief  ~CartesianTask default deconstructor.
    */

    ~CartesianTask();
    /**
     * @brief Method to set the task parameters.
     * @param taskParameters.
     */
    void SetTaskParameter(TaskParameter taskParameters);
    /**
     * @brief  Method used to set the control vector reference in case of equality tasks.
     * @param xReference control vector reference.
     */
    void SetControlVectorReference(Eigen::VectorXd xReference);
    /**
     * @brief  Method returning the task parameter.
     * @return  task parameter.
     */
    const TaskParameter& GetTaskParameter();
    /**
     * @brief  Method used to set the bell shape parameter if the task is of type inequality (either increasing or decreasing).
     * @param bellShapedParameter struct.
     */
    void SetBellShapedParameter(BellShapedParameter bellShapedParameters);
    /**
     * @brief Method returning the bell shape parameter.
     * @return  BellShapedParameter struct of the task.
     */
    const BellShapedParameter& GetBellShapedParameter();

    /**
     * @brief  Method used in order to make the task one dimensional, i.e. project the jacobian along the error direction.
     */
    void SetOneDimensional();
    /**
    * @brief Method returning the task control variable.
    * @return Orientation error expressed wrt to the robot frame .
    */
    Eigen::Vector3d GetControlVariable();
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
           << "x_ \n"
           << "\033[0m" << cartesianTask.x_ << "\n"
           << "\033[0m" << cartesianTask.taskParameter_ << "\n"
           << "\033[1;37m"
           << "Use Error Norm  \n"
           << "\033[0m" << cartesianTask.useErrorNorm_ << "\n";

        if (cartesianTask.taskType_ == CartesianTaskType::InequalityDecreasing) {
            os << "\033[1;37m"
               << "DECREASING bell shape parameters\n"
               << "\033[0m" << cartesianTask.bellShapeParameter_ << "\n";
        }
        if (cartesianTask.taskType_ == CartesianTaskType::InequalityIncreasing) {
            os << "\033[1;37m"
               << "INCREASING bell shape parameters\n"
               << "\033[0m" << cartesianTask.bellShapeParameter_ << "\n";
        }
        if (cartesianTask.taskType_ == CartesianTaskType::Equality) {
            os << "\033[1;37m"
               << "EQUALITY control reference value \n"
               << "\033[0m" << cartesianTask.xReference_ << "\n";
        }
        return os;
    }

protected:
    //void ChangeObserver();
    /**
     * @brief  Implementation of the pure virtual method of the base class Task used to update the internal activation function.
     * Such method must be called in the Update method.
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
     * @brief  Method use to project the jacobian along the error direction. Such method must be called in the Update() method in order to allow the change
     * from 3 dimensional task to 1 dimensional task.
     */
    void UseErrorNormJacobian();
    /**
     * @brief  Method used to check the initialization, hence that all the task parameters have been initializated before updating the task.\n
     * Such meethod must be called in the Update() method before any other method.
     * @note An exception is thrown if the task parameter has not been initialized yet.
     */
    void CheckInitialization() throw(ExceptionWithHow);
    /**
     * @brief  Method to set the task control variable.
     * @details Protected method to be used in the derived task in order to set the task control variable.
     * @param x the task control vector e.g. the distance, the misalignment vector...
     * @note such method must be called in the update method in the derived class in order to instantiate the control variable at each control loop.
     */
    void SetControlVariable(Eigen::Vector3d x);

    //Eigen::MatrixXd JObserver_;//!< The observer jacobian wrt to inertial frame
    BellShapedParameter bellShapeParameter_; //!< The bell shape struct
    TaskParameter taskParameter_; //!< The task parameter struct
    bool useErrorNorm_{ false }; //!< Boolean stating whether project the jacobian along the error direction
    bool initializedTaskParameter_{ false }; //!< Boolean stating whether the task parameter have been initialized
    bool initializedBellShapeParameter_{ false }; //!< Boolean stating whether the bell shaped parameters have been initialized
    CartesianTaskType taskType_; //!< Enum stating whether the task type is either inequality increasing, inequality decreasing or equality
    bool referenceControlVector_; //!< Boolean stating whether the reference control vector has been initialized

private:
    Eigen::Vector3d x_; //!< The control vector
    Eigen::VectorXd xReference_; //!< The control vector reference
};
}
