#include "tpik/ReactiveTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

// public

ReactiveTask::ReactiveTask(const std::string ID, int DoF, TaskType taskType)
    : Task(ID, 3, DoF)
    , taskParameter_{ 0.0, false, 0.0 }
    , taskType_(taskType)
    , initializedTaskParameter_(false)
    , initializedBellShapeParameter_(false)
{
    xReference_.setZero(taskSpace_);
    x_.setZero(taskSpace_);
}

ReactiveTask::~ReactiveTask() {}

void ReactiveTask::CheckInitialization() throw(ExceptionWithHow)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[ReactiveTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
    if (taskType_ == TaskType::InequalityLessThan || taskType_ == TaskType::InequalityGreaterThan || taskType_ == TaskType::InequalityInBetween) {
        if (!initializedBellShapeParameter_) {
            NotInitialziedTaskParameterException notInitializedBellShapeIncreasing;
            std::string how = "[ReactiveTask] Not initialized incresingBellShape struct" + ID_;
            notInitializedBellShapeIncreasing.SetHow(how);
            throw(notInitializedBellShapeIncreasing);
        }

        if ((taskType_ == TaskType::InequalityGreaterThan && increasingBellShapeParameter_.xmax.size() != taskSpace_)
            || (taskType_ == TaskType::InequalityLessThan && decreasingBellShapeParameter_.xmax.size() != taskSpace_)
            || (taskType_ == TaskType::InequalityInBetween && decreasingBellShapeParameter_.xmax.size() != taskSpace_ && increasingBellShapeParameter_.xmax.size() != taskSpace_)) {

            NotInitialziedTaskParameterException wrongBellShapeIcreasingSize;
            std::string how = "[ReactiveTask] Wrong size incresingBellShape struct, expectedSize = " + std::to_string(taskSpace_) + " task " + ID_;
            wrongBellShapeIcreasingSize.SetHow(how);
            throw(wrongBellShapeIcreasingSize);
        }
    }
}

void ReactiveTask::UpdateInternalActivationFunction()
{
    if (taskType_ == TaskType::InequalityGreaterThan) {
        for (int i = 0; i < taskSpace_; i++) {
            Ai_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)));
        }
    } else if (taskType_ == TaskType::InequalityLessThan) {
        for (int i = 0; i < taskSpace_; i++) {
            Ai_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)));
        }
    } else if (taskType_ == TaskType::InequalityInBetween) {
        for (int i = 0; i < taskSpace_; i++) {
            Ai_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)))
                + rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)));
        }
    } else if (taskType_ == TaskType::Equality) {
        Ai_.setIdentity();
    }
}

void ReactiveTask::UpdateReference()
{
    if (taskType_ == TaskType::InequalityLessThan) {
        x_dot_ = taskParameter_.gain * (decreasingBellShapeParameter_.xmax - x_);

    } else if (taskType_ == TaskType::InequalityGreaterThan) {
        x_dot_ = taskParameter_.gain * (increasingBellShapeParameter_.xmin - x_);

    } else if (taskType_ == TaskType::InequalityInBetween) {
        Eigen::VectorXd desired = ((increasingBellShapeParameter_.xmax - decreasingBellShapeParameter_.xmax) / 2) + decreasingBellShapeParameter_.xmax;
        x_dot_ = taskParameter_.gain * (desired - x_);

    } else if (taskType_ == TaskType::Equality) {
        x_dot_ = taskParameter_.gain * (xReference_ - x_);
    }
}

void ReactiveTask::SaturateReference() { rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_); }

void ReactiveTask::SaturateReferenceComponentWise()
{
    for (int i = 0; i < taskSpace_; i++) {
        Eigen::VectorXd x_dot_element(1);
        x_dot_element(0) = x_dot_(i);
        rml::SaturateScalar(taskParameter_.saturation, x_dot_element(0));
        x_dot_(i) = x_dot_element(0);
    }
}

void ReactiveTask::ConfigFromFile(libconfig::Config& confObj)
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

            if (taskType_ == TaskType::InequalityLessThan) {

                const libconfig::Setting& bellShapedParam = task["decreasingBellShaped"];
                decreasingBellShapeParameter_.ConfigureFromFile(bellShapedParam);
                initializedBellShapeParameter_ = true;

            } else if (taskType_ == TaskType::InequalityGreaterThan) {

                const libconfig::Setting& bellShapedParam = task["increasingBellShaped"];
                increasingBellShapeParameter_.ConfigureFromFile(bellShapedParam);
                initializedBellShapeParameter_ = true;

            } else if (taskType_ == TaskType::InequalityInBetween) {

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
}
