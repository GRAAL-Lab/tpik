#include "tpik/CartesianTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {
CartesianTask::CartesianTask(const std::string ID, int DoF, TaskType taskType)
    : Task(ID, 3, DoF)
    , initializedTaskParameter_(false)
    , initializedIncreasingBellShapeParameter_(false)
    , taskType_(taskType)
{
    JObserver_.setZero(6, DoF_);
    useErrorNorm_ = false;
    J_.setZero(6, DoF_);
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

void CartesianTask::SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShapedParameters)
{
    increasingBellShape_ = increasingBellShapedParameters;
    initializedIncreasingBellShapeParameter_ = true;
}

const BellShapedParameter& CartesianTask::GetIncreasingBellShapedParameter()
{
    return increasingBellShape_;
}

void CartesianTask::CheckInitialization() throw(ExceptionWithHow)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[CartesianTask] Not initialized taskParameter struct, use SetTaskParameter() for task "+ ID_ ;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
    if (taskType_ == Inequality) {
        if (!initializedIncreasingBellShapeParameter_) {
            NotInitialziedTaskParameterException notInitializedBellShapeIncreasing;
            std::string how = "[CartesianTask] Not initialized incresingBellShape struct, use SetIncreasingBellShapedParameter() for task "+ ID_ ;
             notInitializedBellShapeIncreasing.SetHow(how);
            throw( notInitializedBellShapeIncreasing);
        }
        if(increasingBellShape_.xmax.size()!=taskSpace_){
            NotInitialziedTaskParameterException wrongBellShapeIcreasingSize;
            std::string how = "[CartesianTask] Wrong size incresingBellShape struct, task space = " + std::to_string(taskSpace_)+" use SetIncreasingBellShapedParameter() for task "+ ID_ ;
             wrongBellShapeIcreasingSize.SetHow(how);
            throw( wrongBellShapeIcreasingSize);
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

void CartesianTask::UpdateInternalActivationFunction()
{
    if (taskType_ == Inequality) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::IncreasingBellShapedFunction(increasingBellShape_.xmin(0), increasingBellShape_.xmax(0), 0, 1, std::fabs(error_.norm()));
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShape_.xmin(i), increasingBellShape_.xmax(i), 0, 1, std::fabs(error_(i)));
            }
        }

    } else if (taskType_ == Equality) {
        Ai_.setIdentity();
    }
}

void CartesianTask::UpdateReference()
{
    if (useErrorNorm_) {
        x_dot_(0) = taskParameter_.gain * (error_.norm());
    } else {
        x_dot_ = taskParameter_.gain * (error_);
    }
}

void CartesianTask::SaturateReference()
{

    rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
}
}
