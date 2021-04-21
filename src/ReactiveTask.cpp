#include "tpik/ReactiveTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

ReactiveTask::ReactiveTask(const std::string ID, int taskSpace, int DoF, tpik::TaskOption taskOption)
    : Task(ID, taskSpace, DoF)
    , taskParameter_ { 0.0, 0.0, false, 0.0 }
    , initializedTaskParameter_ { false }
    , isLessThanParamsInizialized_ { false }
    , isGreaterThanParamsInizialized_ { false }
    , isTaskTypeSet_ { false }
    , taskOption_ { taskOption }
    , saturateRaferenceRateComponentWise_ { false }
{
    if (taskOption_ == tpik::TaskOption::UseErrorNorm) {
        taskSpace_ = 1;
    }

    x_ = Eigen::VectorXd::Zero(taskSpace_);
    x_dot_bar_ = Eigen::VectorXd::Zero(taskSpace_);
    x_bar_ = Eigen::VectorXd::Zero(taskSpace_);
    decreasingBellShapeParameter_.xmax = Eigen::VectorXd::Zero(taskSpace_);
    decreasingBellShapeParameter_.xmin = Eigen::VectorXd::Zero(taskSpace_);
    increasingBellShapeParameter_.xmax = Eigen::VectorXd::Zero(taskSpace_);
    increasingBellShapeParameter_.xmin = Eigen::VectorXd::Zero(taskSpace_);
    AgreaterThan_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);
    AlessThan_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);
    Ai_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);

    Aexternal_ = Eigen::MatrixXd::Identity(taskSpace_, taskSpace_);
}

ReactiveTask::~ReactiveTask() { }

void ReactiveTask::CheckInitialization() noexcept(false)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException e;
        std::string how = "[ReactiveTask] Not initialized taskParameter struct, use TaskParameter() for task ";
        how.append(ID_);
        e.SetHow(how);
        throw(e);
    }
    if (!isTaskTypeSet_) {
        NotInitialziedTaskParameterException e;
        std::string how = "[ReactiveTask] Not set the task type, use Type() for task ";
        how.append(ID_);
        e.SetHow(how);
        throw(e);
    }

    if (taskType_ == TaskType::Inequality) {
        if (!isLessThanParamsInizialized_ && !isGreaterThanParamsInizialized_) {
            NotInitialziedTaskParameterException e;
            std::string how = "[ReactiveTask] Not greater/less than params init for Inequality task, use GreaterThanParams()/LessThanParams() for task ";
            how.append(ID_);
            e.SetHow(how);
            throw(e);
        }

        if (taskType_ == TaskType::Inequality && isLessThanParamsInizialized_ && increasingBellShapeParameter_.xmax.size() != taskSpace_) {
            NotInitialziedTaskParameterException e;
            std::string how = "[ReactiveTask] Wrong size greater than param, expectedSize = ";
            how.append(std::to_string(taskSpace_)).append(" task ").append(ID_);
            e.SetHow(how);
            throw(e);
        }

        if (taskType_ == TaskType::Inequality && isGreaterThanParamsInizialized_ && decreasingBellShapeParameter_.xmax.size() != taskSpace_) {
            NotInitialziedTaskParameterException e;
            std::string how = "[ReactiveTask] Wrong size less than params, expectedSize = ";
            how.append(std::to_string(taskSpace_)).append(" task ").append(ID_);
            e.SetHow(how);
            throw(e);
        }
    }
}

void ReactiveTask::Update()
{
    UpdateReference();
    Task::Update();
    SaturateReferenceRate();
}

void ReactiveTask::UpdateInternalActivationFunction()
{
    if (taskType_ == TaskType::Inequality) {
        if (taskOption_ == tpik::TaskOption::ActiveOnNorm) {
            if (isLessThanParamsInizialized_)
                AlessThan_ = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(0), increasingBellShapeParameter_.xmax(0), 0.0, 1.0, ((x_bar_ - x_).norm())) * AlessThan_.setIdentity();

            if (isGreaterThanParamsInizialized_)
                AgreaterThan_ = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(0), decreasingBellShapeParameter_.xmax(0), 0.0, 1.0, ((x_bar_ - x_).norm())) * AgreaterThan_.setIdentity();

        } else if (taskOption_ == tpik::TaskOption::UseErrorNorm) {
            for (int i = 0; i < taskSpace_; i++) {
                if (isLessThanParamsInizialized_)
                    AlessThan_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_.norm()));

                if (isGreaterThanParamsInizialized_)
                    AgreaterThan_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_.norm()));
            }
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                if (isLessThanParamsInizialized_)
                    AlessThan_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)));

                if (isGreaterThanParamsInizialized_)
                    AgreaterThan_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)));
            }
        }
        Ai_ = AgreaterThan_ + AlessThan_;

    } else if (taskType_ == TaskType::Equality) {
        Ai_.setIdentity();
    }
}

void ReactiveTask::UpdateReference()
{
    if (taskType_ == TaskType::Inequality) {
        for (int i = 0; i < taskSpace_; i++) {
            if (AgreaterThan_(i) > 0) {
                x_bar_(i) = decreasingBellShapeParameter_.xmax(i);
            } else if (AlessThan_(i) > 0) {
                x_bar_(i) = increasingBellShapeParameter_.xmin(i);
            }
        }
    }
}

void ReactiveTask::UpdateReferenceRate()
{
    for (int i = 0; i < taskSpace_; i++) {
        if (Ai_.diagonal()(i) > 0) {
            if (taskOption_ == tpik::TaskOption::UseErrorNorm) {
                x_dot_bar_(i) = taskParameter_.gain * (x_bar_(i) - x_.norm());
            } else {
                x_dot_bar_(i) = taskParameter_.gain * (x_bar_(i) - x_(i));
            }
        } else {
            x_dot_bar_(i) = 0;
        }
    }
}

void ReactiveTask::UpdateJacobian()
{
    if (taskOption_ == tpik::TaskOption::UseErrorNorm) {
        if (x_.norm() == 0.0) {
            J_.resize(1, dof_);
            J_.setZero();
        } else {
            J_ = x_.normalized().transpose() * J_;
        }
    }
}

void ReactiveTask::SaturateReferenceRate()
{
    if (saturateRaferenceRateComponentWise_) {
        for (int i = 0; i < taskSpace_; i++) {
            rml::SaturateScalar(taskParameter_.saturation, x_dot_bar_(i));
        }
    } else {
        rml::SaturateVector(taskParameter_.saturation, x_dot_bar_);
    }
}

bool ReactiveTask::ConfigFromFile(libconfig::Config& confObj) noexcept(false)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& tasks = root["tasks"];

    //Check if the task name exist in the conf file.
    assert(tasks.exists(ID_));

    const libconfig::Setting& task = tasks.lookup(ID_);
    if (taskParameter_.ConfigureFromFile(task)) {
        initializedTaskParameter_ = true;
    }

    int tmpType;

    if (ctb::GetParam(task, tmpType, "type")) {
        taskType_ = static_cast<tpik::TaskType>(tmpType);
        isTaskTypeSet_ = true;
    }

    if (taskType_ == TaskType::Inequality) {

        if (task.exists("greaterThanParams")) {
            const libconfig::Setting& decbellShapedParam = task.lookup("greaterThanParams");
            decreasingBellShapeParameter_.ConfigureFromFile(decbellShapedParam);
            isGreaterThanParamsInizialized_ = true;
        }

        if (task.exists("lessThanParams")) {
            const libconfig::Setting& incbellShapedParam = task.lookup("lessThanParams");
            increasingBellShapeParameter_.ConfigureFromFile(incbellShapedParam);
            isLessThanParamsInizialized_ = true;
        }
    }

    if (task.exists("saturateRaferenceRateComponentWise")) {
        task.lookupValue("saturateRaferenceRateComponentWise", saturateRaferenceRateComponentWise_);
    }

    return true;
}
}
