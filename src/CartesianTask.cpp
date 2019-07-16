#include "tpik/CartesianTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

// public

CartesianTask::CartesianTask(const std::string ID, int DoF, CartesianTaskType taskType, ProjectorType projectorType)
    : Task(ID, 3, DoF)
    , initializedTaskParameter_(false)
    , initializedBellShapeParameter_(false)
    , taskType_(taskType)
    , referenceControlVector_(false)
    , projectorType_(projectorType)
    , activateOnNorm_(false)
{
    useErrorNorm_ = false;
}

CartesianTask::~CartesianTask() {}

void CartesianTask::SetTaskParameter(TaskParameter taskParameters)
{
    taskParameter_ = taskParameters;
    isActive_ = taskParameter_.taskEnable;
    initializedTaskParameter_ = true;
}

const TaskParameter& CartesianTask::GetTaskParameter() { return taskParameter_; }

void CartesianTask::SetBellShapedParameter(BellShapedParameter increasingBellShapedParameters)
{
    bellShapeParameter_ = increasingBellShapedParameters;
    initializedBellShapeParameter_ = true;
}

void CartesianTask::SetBellShapedParameterScalar(double bellShapedParametersXmin, double bellShapedParametersXmax)
{
    Eigen::VectorXd bellShapeXMaxVector;
    bellShapeXMaxVector.resize(1);
    bellShapeXMaxVector(0) = bellShapedParametersXmax;
    Eigen::VectorXd bellShapeXMinVector;
    bellShapeXMinVector.resize(1);
    bellShapeXMinVector(0) = bellShapedParametersXmin;
    bellShapeParameter_.xmax = bellShapeXMaxVector;
    bellShapeParameter_.xmin = bellShapeXMinVector;
    activateOnNorm_ = true;
    initializedBellShapeParameter_ = true;
}

void CartesianTask::SetBellShapedParameterInBetweenScalar(double increasingBellShapedParametersXmin,
    double increasingbellShapedParametersXmax, double decreasingBellShapedParametersXmin,
    double decreasingBellShapedParametersXmax)
{
    Eigen::VectorXd increasingBellShapeXMaxVector;
    increasingBellShapeXMaxVector.resize(1);
    increasingBellShapeXMaxVector(0) = increasingbellShapedParametersXmax;
    Eigen::VectorXd increasingBellShapeXMinVector;
    increasingBellShapeXMinVector.resize(1);
    increasingBellShapeXMinVector(0) = increasingBellShapedParametersXmin;

    Eigen::VectorXd decreasingBellShapeXMaxVector;
    decreasingBellShapeXMaxVector.resize(1);
    decreasingBellShapeXMaxVector(0) = decreasingBellShapedParametersXmax;
    Eigen::VectorXd decreasingBellShapeXMinVector;
    decreasingBellShapeXMinVector.resize(1);
    decreasingBellShapeXMinVector(0) = decreasingBellShapedParametersXmin;

    bellShapeParameter_.xmax = increasingBellShapeXMaxVector;
    bellShapeParameter_.xmin = increasingBellShapeXMinVector;
    inequalityDecreasingBellShapeParameter_.xmax = decreasingBellShapeXMaxVector;
    inequalityDecreasingBellShapeParameter_.xmin = decreasingBellShapeXMinVector;
    initializedBellShapeParameter_ = true;
    activateOnNorm_ = true;
}

void CartesianTask::SetBellShapedParameterInBetween(
    BellShapedParameter increasingBellShapedParameters, BellShapedParameter decreasingBellShapedParameter)
{
    inequalityDecreasingBellShapeParameter_ = decreasingBellShapedParameter;
    bellShapeParameter_ = increasingBellShapedParameters;
    initializedBellShapeParameter_ = true;
}

const BellShapedParameter& CartesianTask::GetBellShapedParameter()
{

    if (taskType_ == CartesianTaskType::Equality) {
        std::cerr << "[WARNING] asking bell shape parameter of an equality task " << ID_ << std::endl;
    }

    return bellShapeParameter_;
}

const BellShapedParameter& CartesianTask::GetInBetweenDecreasingBellShapedParameter()
{

    if (!(taskType_ == CartesianTaskType::InequalityInBetween)) {
        std::cerr << "[WARNING] the task is not an inequality in between task " << ID_ << std::endl;
    }
    return inequalityDecreasingBellShapeParameter_;
}

Eigen::VectorXd CartesianTask::GetControlVariable()
{
    // if (useErrorNorm_) {
    //     Eigen::VectorXd x;
    //     x.resize(1);
    //     x(0) = x_.norm();
    //     return x;
    // } else {
    return x_;
    //}
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

void CartesianTask::SetProjectorParameters(Eigen::Vector3d vector, std::string frameID)
{
    normalProjector_ = vector;
    frameIDProjector_ = frameID;
}
// protected

void CartesianTask::CheckInitialization() throw(ExceptionWithHow)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how
            = "[CartesianTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
    if (taskType_ == CartesianTaskType::InequalityDecreasing || taskType_ == CartesianTaskType::InequalityIncreasing
        || taskType_ == CartesianTaskType::InequalityInBetween) {
        if (!initializedBellShapeParameter_) {
            NotInitialziedTaskParameterException notInitializedBellShapeIncreasing;
            std::string how = "[CartesianTask] Not initialized incresingBellShape struct, use "
                              "SetIncreasingBellShapedParameter() for task "
                + ID_;
            notInitializedBellShapeIncreasing.SetHow(how);
            throw(notInitializedBellShapeIncreasing);
        }
        int bellShapeExpectedSize = taskSpace_;
        if (activateOnNorm_) {
            bellShapeExpectedSize = 1;
        }
        if (bellShapeParameter_.xmax.size() != bellShapeExpectedSize) {

            NotInitialziedTaskParameterException wrongBellShapeIcreasingSize;
            std::string how = "[CartesianTask] Wrong size incresingBellShape struct, expectedSize = "
                + std::to_string(bellShapeExpectedSize) + " use SetIncreasingBellShapedParameter() for task " + ID_;
            wrongBellShapeIcreasingSize.SetHow(how);
            throw(wrongBellShapeIcreasingSize);
        }
    }

    else if (taskType_ == CartesianTaskType::Equality) {
        if (!referenceControlVector_) {
            xReference_.setZero(taskSpace_);
        }
    }
}

void CartesianTask::SetControlVectorReference(Eigen::VectorXd xReference)
{
    if (xReference.size() != taskSpace_) {
        std::cerr << "wrong size xReference, task space = " << taskSpace_ << " task ID = " << ID_ << std::endl;
        referenceControlVector_ = false;
    } else {
        xReference_ = xReference;
        referenceControlVector_ = true;
    }
}

void CartesianTask::UseErrorNormJacobian()
{
    if (useErrorNorm_) {
        if (x_.norm() == 0.0) {
            J_.resize(1, DoF_);
            J_.setZero();
        } else {
            J_ = (x_.transpose() / x_.norm()) * J_;
        }
    }
}

void CartesianTask::UpdateInternalActivationFunction()
{
    Ai_.setZero(taskSpace_, taskSpace_);
    if (taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::IncreasingBellShapedFunction(
                bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm())); // removed fabs
        } else if (activateOnNorm_) {
            double a = rml::IncreasingBellShapedFunction(
                bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()));
            Ai_ = a * Eigen::Matrix3d::Identity();
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::IncreasingBellShapedFunction(
                    bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i))); // removed fabs
            }
        }

    } else if (taskType_ == CartesianTaskType::InequalityDecreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::DecreasingBellShapedFunction(
                bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm())); // removed fabs
        } else if (activateOnNorm_) {
            double a = rml::DecreasingBellShapedFunction(
                bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()));
            Ai_ = a * Eigen::Matrix3d::Identity();
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::DecreasingBellShapedFunction(
                    bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i))); // removed fabs
            }
        }
    } else if (taskType_ == CartesianTaskType::InequalityInBetween) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::DecreasingBellShapedFunction(inequalityDecreasingBellShapeParameter_.xmin(0),
                            inequalityDecreasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()))
                + rml::IncreasingBellShapedFunction(bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0.0, 1.0,
                            (x_.norm())); // removed fabs
        } else if (activateOnNorm_) {
            double a = rml::DecreasingBellShapedFunction(inequalityDecreasingBellShapeParameter_.xmin(0),
                           inequalityDecreasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()))
                + rml::IncreasingBellShapedFunction(
                           bellShapeParameter_.xmin(0), bellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()));
            Ai_ = a * Eigen::Matrix3d::Identity();
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::DecreasingBellShapedFunction(inequalityDecreasingBellShapeParameter_.xmin(i),
                                inequalityDecreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)))
                    + rml::IncreasingBellShapedFunction(bellShapeParameter_.xmin(i), bellShapeParameter_.xmax(i), 0.0,
                                1.0, (x_(i))); // removed fabs
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
        } else if (activateOnNorm_) {
            x_dot_ = taskParameter_.gain * (bellShapeParameter_.xmax(0) - x_.norm())
                * Eigen::VectorXd::Ones(taskSpace_); // TODO
        } else {
            x_dot_ = taskParameter_.gain * (bellShapeParameter_.xmax - x_);
        }
    } else if (taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * (bellShapeParameter_.xmin(0) - x_.norm());
        } else if (activateOnNorm_) {
            x_dot_ = taskParameter_.gain * (bellShapeParameter_.xmin(0) - x_.norm())
                * Eigen::VectorXd::Ones(taskSpace_); // TODO
        } else {
            x_dot_ = taskParameter_.gain * (bellShapeParameter_.xmin - x_);
        }
    } else if (taskType_ == CartesianTaskType::InequalityInBetween) {

        if (useErrorNorm_) {
            double desired = ((bellShapeParameter_.xmax(0) - inequalityDecreasingBellShapeParameter_.xmax(0)) / 2)
                + inequalityDecreasingBellShapeParameter_.xmax(0);
            x_dot_(0) = taskParameter_.gain * (desired - x_.norm());
        } else if (activateOnNorm_) {
            double desired = ((bellShapeParameter_.xmax(0) - inequalityDecreasingBellShapeParameter_.xmax(0)) / 2)
                + inequalityDecreasingBellShapeParameter_.xmax(0);
            x_dot_ = taskParameter_.gain * (desired - x_.norm()) * Eigen::VectorXd::Ones(taskSpace_);

        } else {
            Eigen::VectorXd desired = ((bellShapeParameter_.xmax - inequalityDecreasingBellShapeParameter_.xmax) / 2)
                + inequalityDecreasingBellShapeParameter_.xmax;
            x_dot_ = taskParameter_.gain * (desired - x_);
        }
    } else if (taskType_ == CartesianTaskType::Equality) {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * (xReference_(0) - x_.norm());
        } else if (activateOnNorm_) {
            x_dot_ = taskParameter_.gain * (xReference_(0) - x_.norm()) * Eigen::VectorXd::Ones(taskSpace_);
        }
        {
            x_dot_ = taskParameter_.gain * (xReference_ - x_);
        }
    }
}

void CartesianTask::SaturateReference() { rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_); }

void CartesianTask::SaturateReferenceComponentWise()
{
    for (int i = 0; i < taskSpace_; i++) {
        Eigen::VectorXd x_dot_element(1);
        x_dot_element(0) = x_dot_(i);
        rml::SaturateScalar(taskParameter_.saturation, x_dot_element(0));
        x_dot_(i) = x_dot_element(0);
    }
}

void CartesianTask::UpdateProjector()
{

    switch (projectorType_) {
    case (ProjectorType::Default): {
        PBodyFrame_ = Eigen::Matrix3d::Identity();

    } break;
    case (ProjectorType::OnLine): {
        Eigen::Vector3d projectorBodyFrame = bodyFrame_T_parameterProjector_.GetRotMatrix() * normalProjector_;
        PBodyFrame_ = ((projectorBodyFrame * projectorBodyFrame.transpose()));
    } break;

    case (ProjectorType::OnPlane): {
        Eigen::Vector3d projectorBodyFrame = bodyFrame_T_parameterProjector_.GetRotMatrix() * normalProjector_;
        PBodyFrame_ = (Eigen::Matrix3d::Identity() - projectorBodyFrame * projectorBodyFrame.transpose());
    } break;
    };
}
// private

void CartesianTask::SetControlVariable(Eigen::VectorXd x) { x_ = x; }

CartesianTaskType CartesianTask::GetType() { return taskType_; }

ProjectorType CartesianTask::GetProjectorType() { return projectorType_; }

Eigen::VectorXd CartesianTask::GetControlVariableReference() { return xReference_; }
}

// void CartesianTask::ChangeObserver()
//{
//    J_ = rml::ChangeJacobianObserver(J_, JObserver_, x_);
//}
