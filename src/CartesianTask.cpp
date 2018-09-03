#include "tpik/CartesianTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

// public

CartesianTask::CartesianTask(const std::string ID, int DoF, CartesianTaskType taskType)
    : Task(ID, 3, DoF)
    , initializedTaskParameter_(false)
    , initializedBellShapeParameter_(false)
    , taskType_(taskType)
    , referenceControlVector_(false)
{
    // JObserver_.setZero(taskSpace_, DoF_);
    useErrorNorm_ = false;
    xReference_.resize(taskSpace_);
    xReference_.setZero();
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
    if (taskType_ == CartesianTaskType::Equality) {
        std::cerr << "[WARNING] asking bell shape parameter of an equality task " << ID_ << std::endl;
        bellShapeParameter_.xmax.setZero(taskSpace_);
        bellShapeParameter_.xmin.setZero(taskSpace_);
    }
    return bellShapeParameter_;
}

Eigen::Vector3d CartesianTask::GetControlVariable()
{
    return x_;
}

void CartesianTask::SetOneDimensional()
{
    useErrorNorm_ = true;
    taskSpace_ = 1;
    Ai_.setZero(taskSpace_, taskSpace_);
    x_dot_.setZero(taskSpace_);
    J_.setZero(taskSpace_, DoF_);
    xReference_.setZero(taskSpace_);
}

// protected

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
    } else if (taskType_ == CartesianTaskType::Equality) {
        if (!referenceControlVector_) {
            std::cerr << "[CartesianTask] Not initialized control vector reference, using default value 0 " << std::endl;
        }
    }
}

void CartesianTask::SetControlVectorReference(Eigen::VectorXd xReference)
{
    xReference_ = xReference;
    referenceControlVector_ = true;
}

void CartesianTask::UseErrorNormJacobian()
{
    if (useErrorNorm_) {
        if (x_.norm() == 0.0) {
            J_.resize(1, 1);
            J_(0, 0) = 0.0;
        } else {
            J_ = (x_.transpose() / x_.norm()) * J_;
        }
    }
}

void CartesianTask::UpdateInternalActivationFunction()
{
    if (taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::IncreasingBellShapedFunction(bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0, 1, std::fabs(x_.norm()));
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::IncreasingBellShapedFunction(bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0, 1, std::fabs(x_(i)));
            }
        }

    } else if (taskType_ == CartesianTaskType::InequalityDecreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::DecreasingBellShapedFunction(bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0, 1, std::fabs(x_.norm()));
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::DecreasingBellShapedFunction(bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0, 1, std::fabs(x_(i)));
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
            x_dot_(0) = taskParameter_.gain * (bellShapeParameter_.xmax(0) - x_.norm());
        } else {
            x_dot_ = taskParameter_.gain * (bellShapeParameter_.xmax - x_);
        }
    } else if (taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * ( x_.norm() - bellShapeParameter_.xmin(0));
        } else {
            x_dot_ = taskParameter_.gain * ( x_ - bellShapeParameter_.xmin );
        }
    } else  {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * (xReference_(0) - x_.norm());
        } else {
            x_dot_ = taskParameter_.gain * (xReference_ - x_);
        }
    }
}

void CartesianTask::SaturateReference()
{
    rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
}

// private

void CartesianTask::SetControlVariable(Eigen::Vector3d x)
{
    x_ = x;
}

CartesianTaskType CartesianTask::GetType()
{
    return taskType_;
}
}

//void CartesianTask::ChangeObserver()
//{
//    J_ = rml::ChangeJacobianObserver(J_, JObserver_, x_);
//}
