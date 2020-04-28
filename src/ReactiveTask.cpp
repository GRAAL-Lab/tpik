#include "tpik/ReactiveTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

// public

ReactiveTask::ReactiveTask(const std::string ID, int taskSpace, int DoF, TaskType taskType)
    : Task(ID, taskSpace, DoF)
    , taskParameter_{ 0.0, false, 0.0 }
    , taskType_{ taskType }
    , initializedTaskParameter_{ false }
    , isLessThanParamsInizialized{ false }
    , isGreaterThanParamsInizialized{ false }
    , useErrorNorm_{ false }
    , useActiveOnNorm_{ false }
    , saturareRateComponentWise_{ false }
{
    x_bar_.setZero(taskSpace_);
    x_.setZero(taskSpace_);
    decreasingBellShapeParameter_.xmax.setZero(taskSpace_);
    decreasingBellShapeParameter_.xmin.setZero(taskSpace_);
    increasingBellShapeParameter_.xmax.setZero(taskSpace_);
    increasingBellShapeParameter_.xmin.setZero(taskSpace_);
    AgreaterThan_.setZero(taskSpace_, taskSpace_);
    AlessThan_.setZero(taskSpace_, taskSpace_);
}

ReactiveTask::~ReactiveTask() {}

void ReactiveTask::CheckInitialization() noexcept(false)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException e;
        std::string how = "[ReactiveTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
        e.SetHow(how);
        throw(e);
    }
    if (taskType_ == TaskType::Inequality) {
        if (!isLessThanParamsInizialized || !isGreaterThanParamsInizialized) {
            NotInitialziedTaskParameterException e;
            std::string how = "[ReactiveTask] Not initialized incresingBellShape struct" + ID_;
            e.SetHow(how);
            throw(e);
        }

        if ((taskType_ == TaskType::Inequality && increasingBellShapeParameter_.xmax.size() != taskSpace_)
            || (taskType_ == TaskType::Inequality && decreasingBellShapeParameter_.xmax.size() != taskSpace_)) {

            NotInitialziedTaskParameterException e;
            std::string how = "[ReactiveTask] Wrong size incresingBellShape struct, expectedSize = " + std::to_string(taskSpace_) + " task " + ID_;
            e.SetHow(how);
            throw(e);
        }
    }
}

void ReactiveTask::Update()
{
    Task::Update();
    SaturateReferenceRate();
}

void ReactiveTask::UpdateInternalActivationFunction()
{
    if (taskType_ == TaskType::Inequality) {
        if (useActiveOnNorm_) {
            if (isLessThanParamsInizialized)
                AlessThan_ = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(0), increasingBellShapeParameter_.xmax(0), 0.0, 1.0, ((x_bar_ - x_).norm())) * AlessThan_.setIdentity();

            if (isGreaterThanParamsInizialized)
                AgreaterThan_ = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(0), decreasingBellShapeParameter_.xmax(0), 0.0, 1.0, ((x_bar_ - x_).norm())) * AgreaterThan_.setIdentity();

        } else if (useErrorNorm_) {
            for (int i = 0; i < taskSpace_; i++) {
                if (isLessThanParamsInizialized)
                    AlessThan_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_.norm()));

                if (isGreaterThanParamsInizialized)
                    AgreaterThan_(i, i) = rml::DecreasingBellShapedFunction(decreasingBellShapeParameter_.xmin(i), decreasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_.norm()));
            }
        } else {
            for (int i = 0; i < taskSpace_; i++) {
                if (isLessThanParamsInizialized)
                    AlessThan_(i, i) = rml::IncreasingBellShapedFunction(increasingBellShapeParameter_.xmin(i), increasingBellShapeParameter_.xmax(i), 0.0, 1.0, (x_(i)));

                if (isGreaterThanParamsInizialized)
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
            if (useErrorNorm_) {
                x_dot_(i) = taskParameter_.gain * (x_bar_(i) - x_.norm());
            } else {
                x_dot_(i) = taskParameter_.gain * (x_bar_(i) - x_(i));
            }
        } else {
            x_dot_(i) = 0;
        }
    }
}

void ReactiveTask::UpdateJacobian()
{
    if (useErrorNorm_) {
        if (x_.norm() == 0.0) {
            J_.resize(1, dof_);
            J_.setZero();
        } else {
            J_ = (x_.transpose() / x_.norm()) * J_;
        }
    }
}

void ReactiveTask::SaturateReferenceRate()
{
    if (saturareRateComponentWise_) {
        for (int i = 0; i < taskSpace_; i++) {
            rml::SaturateScalar(taskParameter_.saturation, x_dot_(i));
        }
    } else {
        rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
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

            if (taskType_ == TaskType::Inequality) {

                const libconfig::Setting& decbellShapedParam = task["decreasingBellShaped"];
                decreasingBellShapeParameter_.ConfigureFromFile(decbellShapedParam);

                const libconfig::Setting& incbellShapedParam = task["increasingBellShaped"];
                increasingBellShapeParameter_.ConfigureFromFile(incbellShapedParam);
            }

            if (task.lookupValue("useActiveOnNorm", useActiveOnNorm_)) {
            }
            if (task.lookupValue("useErrorNorm", useErrorNorm_)) {

                taskSpace_ = 1;
                Ai_.resize(taskSpace_, taskSpace_);
                x_dot_.resize(taskSpace_);
                J_.resize(taskSpace_, dof_);
                Aexternal_.resize(taskSpace_, taskSpace_);
                Aexternal_ = Eigen::VectorXd::Ones(taskSpace_).asDiagonal();
                x_bar_.resize(taskSpace_);
            }
        }
    }
}
}
