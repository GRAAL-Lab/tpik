#include "tpik/CartesianTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {
CartesianTask::CartesianTask(const std::string ID, int DoF, CartesianTaskType taskType)
    : Task(ID, 3, DoF)
    , initializedTaskParameter_(false)
    , initializedBellShapeParameter_(false)
    , taskType_(taskType)
{
    JObserver_.setZero(3, DoF_);
    useErrorNorm_ = false;
    J_.setZero(3, DoF_);
}

CartesianTask::~CartesianTask()
{
}

void CartesianTask::SetTaskParameter(TaskParameter taskParameters)
{
    taskParameter_ = taskParameters;
    initializedTaskParameter_ = true;
}

const TaskParameter& CartesianTask::GetTaskParameter()
{
    return taskParameter_;
}

void CartesianTask::SetBellShapedParameter(BellShapedParameter increasingBellShapedParameters)
{
    bellShapeParameter_ = increasingBellShapedParameters;
    initializedBellShapeParameter_ = true;
}

const BellShapedParameter& CartesianTask::GetBellShapedParameter()
{
    return bellShapeParameter_;
}

void CartesianTask::CheckInitialization() throw(ExceptionWithHow)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[CartesianTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
    if (taskType_ == CartesianTaskType::InequalityDecreasing || taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (!initializedBellShapeParameter_) {
            NotInitialziedTaskParameterException notInitializedBellShapeIncreasing;
            std::string how = "[CartesianTask] Not initialized incresingBellShape struct, use SetIncreasingBellShapedParameter() for task " + ID_;
            notInitializedBellShapeIncreasing.SetHow(how);
            throw(notInitializedBellShapeIncreasing);
        }
        if (bellShapeParameter_.xmax.size() != taskSpace_) {
            NotInitialziedTaskParameterException wrongBellShapeIcreasingSize;
            std::string how = "[CartesianTask] Wrong size incresingBellShape struct, task space = " + std::to_string(taskSpace_) + " use SetIncreasingBellShapedParameter() for task " + ID_;
            wrongBellShapeIcreasingSize.SetHow(how);
            throw(wrongBellShapeIcreasingSize);
        }
    }
}

void CartesianTask::SetUseErrorNorm()
{
    useErrorNorm_ = true;
    taskSpace_ = 1;
    Ai_.setZero(taskSpace_, taskSpace_);
    x_dot_.setZero(taskSpace_);
}

void CartesianTask::ChangeObserver()
{
    J_ = rml::ChangeJacobianObserver(J_, JObserver_, error_);
}

void CartesianTask::UseErrorNormJacobian(){
    if (useErrorNorm_) {
        if (error_.norm() == 0.0) {
            J_.resize(1,1);
            J_(0, 0) = 0.0;
        } else {
            J_ = (error_.transpose() / error_.norm()) * J_;
        }
    }

}
void CartesianTask::UpdateInternalActivationFunction()
{
    if (taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::IncreasingBellShapedFunction(bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0, 1, std::fabs(error_.norm()));
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::IncreasingBellShapedFunction(bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0, 1, std::fabs(error_(i)));
            }
        }

    } else if (taskType_ == CartesianTaskType::InequalityDecreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::DecreasingBellShapedFunction(bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0, 1, std::fabs(error_.norm()));
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::DecreasingBellShapedFunction(bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0, 1, std::fabs(error_(i)));
            }
        }

    } else if (taskType_ == CartesianTaskType::Equality) {
        Ai_.setIdentity();
    }
}

void CartesianTask::UpdateReference()
{
    if (taskType_ == CartesianTaskType::InequalityDecreasing) {
        if (useErrorNorm_) {
            x_dot_(0) = -taskParameter_.gain * (error_.norm());
        } else {
            x_dot_ = -taskParameter_.gain * (error_);
        }
    } else {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * (error_.norm());
        } else {
            x_dot_ = taskParameter_.gain * (error_);
        }
    }
}

void CartesianTask::SaturateReference()
{

    rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
}
}
