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

void CartesianTask::SetTaskParameter(double gain)
{
    taskParameter_.gain = gain;
}

const TaskParameter& CartesianTask::GetTaskParameter() { return taskParameter_; }

CartesianTaskType CartesianTask::GetType() { return taskType_; }

Eigen::VectorXd CartesianTask::GetControlVariable()
{
    return x_;
}

void CartesianTask::SetActivetedOnNorm()
{
    activateOnNorm_ = true;
    increasingBellShapeParameter_.resize(1);
    decreasingBellShapeParameter_.resize(1);
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
        std::string how = "[CartesianTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
    if (taskType_ == CartesianTaskType::InequalityDecreasing || taskType_ == CartesianTaskType::InequalityIncreasing || taskType_ == CartesianTaskType::InequalityInBetween) {
        if (!initializedBellShapeParameter_) {
            NotInitialziedTaskParameterException notInitializedBellShapeIncreasing;
            std::string how = "[CartesianTask] Not initialized incresingBellShape struct" + ID_;
            notInitializedBellShapeIncreasing.SetHow(how);
            throw(notInitializedBellShapeIncreasing);
        }

        int bellShapeExpectedSize = taskSpace_;
        if (activateOnNorm_) {
            bellShapeExpectedSize = 1;
        }

        if ((taskType_ == CartesianTaskType::InequalityIncreasing && increasingBellShapeParameter_.xmax.size() != bellShapeExpectedSize)
            || (taskType_ == CartesianTaskType::InequalityDecreasing && decreasingBellShapeParameter_.xmax.size() != bellShapeExpectedSize)
            || (taskType_ == CartesianTaskType::InequalityInBetween && decreasingBellShapeParameter_.xmax.size() != bellShapeExpectedSize && increasingBellShapeParameter_.xmax.size() != bellShapeExpectedSize)) {

            NotInitialziedTaskParameterException wrongBellShapeIcreasingSize;
            std::string how = "[CartesianTask] Wrong size incresingBellShape struct, expectedSize = " + std::to_string(bellShapeExpectedSize) + " task " + ID_;
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
            Ai_(0, 0) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(0), increasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm())); // removed fabs
        } else if (activateOnNorm_) {
            double a = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(0), increasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()));
            Ai_ = a * Eigen::Matrix3d::Identity();
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i))); // removed fabs
            }
        }

    } else if (taskType_ == CartesianTaskType::InequalityDecreasing) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(0), decreasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm())); // removed fabs
        } else if (activateOnNorm_) {
            double a = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(0), decreasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()));
            Ai_ = a * Eigen::Matrix3d::Identity();
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i))); // removed fabs
            }
        }
    } else if (taskType_ == CartesianTaskType::InequalityInBetween) {
        if (useErrorNorm_) {
            Ai_(0, 0) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(0), decreasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()))
                + rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(0), increasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm())); // removed fabs
        } else if (activateOnNorm_) {
            double a = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(0), decreasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()))
                + rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(0), increasingBellShapeParameter_.xmax(0), 0.0, 1.0, (x_.norm()));

            Ai_ = a * Eigen::Matrix3d::Identity();
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                Ai_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)))
                    + rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i))); // removed fabs
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
            x_dot_(0) = taskParameter_.gain * (decreasingBellShapeParameter_.xmax(0) - x_.norm());
        } else if (activateOnNorm_) {
            x_dot_ = taskParameter_.gain * (decreasingBellShapeParameter_.xmax(0) - x_.norm())
                * Eigen::VectorXd::Ones(taskSpace_); // TODO
        } else {
            x_dot_ = taskParameter_.gain * (decreasingBellShapeParameter_.xmax - x_);
        }
    } else if (taskType_ == CartesianTaskType::InequalityIncreasing) {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * (increasingBellShapeParameter_.xmin(0) - x_.norm());
        } else if (activateOnNorm_) {
            x_dot_ = taskParameter_.gain * (increasingBellShapeParameter_.xmin(0) - x_.norm())
                * Eigen::VectorXd::Ones(taskSpace_); // TODO
        } else {
            x_dot_ = taskParameter_.gain * (increasingBellShapeParameter_.xmin - x_);
        }
    } else if (taskType_ == CartesianTaskType::InequalityInBetween) {

        if (useErrorNorm_) {
            double desired = ((increasingBellShapeParameter_.xmax(0) - decreasingBellShapeParameter_.xmax(0)) / 2)
                + decreasingBellShapeParameter_.xmax(0);
            x_dot_(0) = taskParameter_.gain * (desired - x_.norm());
        } else if (activateOnNorm_) {
            double desired = ((increasingBellShapeParameter_.xmax(0) - decreasingBellShapeParameter_.xmax(0)) / 2)
                + decreasingBellShapeParameter_.xmax(0);
            x_dot_ = taskParameter_.gain * (desired - x_.norm()) * Eigen::VectorXd::Ones(taskSpace_);

        } else {
            Eigen::VectorXd desired = ((increasingBellShapeParameter_.xmax - decreasingBellShapeParameter_.xmax) / 2)
                + decreasingBellShapeParameter_.xmax;
            x_dot_ = taskParameter_.gain * (desired - x_);
        }
    } else if (taskType_ == CartesianTaskType::Equality) {
        if (useErrorNorm_) {
            x_dot_(0) = taskParameter_.gain * (xReference_(0) - x_.norm());
        } else if (activateOnNorm_) {
            x_dot_ = taskParameter_.gain * (xReference_(0) - x_.norm()) * Eigen::VectorXd::Ones(taskSpace_);
        } else {
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

void CartesianTask::ConfigFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& tasks = root["tasks"];

    for (int i = 0; i < tasks.getLength(); ++i) {

        const libconfig::Setting& task = tasks[i];

        std::string name;
        ctb::SetParam(task, name, "name");

        if (ID_ == name) {

            taskParameter_.ConfigureFromFile(task);
            initializedTaskParameter_ = true;

            if (taskType_ == CartesianTaskType::InequalityDecreasing) {

                const libconfig::Setting& bellShapedParam = task["decreasingBellShaped"];
                decreasingBellShapeParameter_.ConfigureFromFile(bellShapedParam);
                initializedBellShapeParameter_ = true;

                std::cout << "DEBUG decreasing:" << decreasingBellShapeParameter_ << std::endl;

            } else if (taskType_ == CartesianTaskType::InequalityIncreasing) {

                const libconfig::Setting& bellShapedParam = task["increasingBellShaped"];
                increasingBellShapeParameter_.ConfigureFromFile(bellShapedParam);
                initializedBellShapeParameter_ = true;

            } else if (taskType_ == CartesianTaskType::InequalityInBetween) {

                const libconfig::Setting& increasingBellShapedParam = task["increasingBellShaped"];
                increasingBellShapeParameter_.ConfigureFromFile(increasingBellShapedParam);

                const libconfig::Setting& decreasingBellShapedParam = task["decreasingBellShaped"];
                decreasingBellShapeParameter_.ConfigureFromFile(decreasingBellShapedParam);
                initializedBellShapeParameter_ = true;
            }
            std::cout << "DEBUG taskparam: " << ID_ << ", " << taskParameter_ << std::endl;
        }
    }
}
// private

void CartesianTask::SetControlVariable(Eigen::VectorXd x) { x_ = x; }

ProjectorType CartesianTask::GetProjectorType() { return projectorType_; }

Eigen::VectorXd CartesianTask::GetControlVariableReference() { return xReference_; }
}
