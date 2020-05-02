#include "tpik/ReactiveTask.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

// public
ReactiveTask::ReactiveTask(const std::string ID, int taskSpace, int DoF, tpik::TaskOption taskOption)
    : Task(ID, taskSpace, DoF)
    , taskParameter_{ 0.0, false, 0.0 }
    , initializedTaskParameter_{ false }
    , isLessThanParamsInizialized_{ false }
    , isGreaterThanParamsInizialized_{ false }
    , taskOption_{ taskOption }
    , saturareRateComponentWise_{ false }
{
    if (taskOption_ == tpik::TaskOption::UseErrorNorm) {
        taskSpace_ = 1;
        x_dot_.conservativeResize(taskSpace_);
        Ai_.conservativeResize(taskSpace_, taskSpace_);
        Aexternal_.conservativeResize(taskSpace_, taskSpace_);
    }

    x_bar_ = Eigen::VectorXd::Zero(taskSpace_);
    x_ = Eigen::VectorXd::Zero(taskSpace);
    decreasingBellShapeParameter_.xmax = Eigen::VectorXd::Zero(taskSpace_);
    decreasingBellShapeParameter_.xmin = Eigen::VectorXd::Zero(taskSpace_);
    increasingBellShapeParameter_.xmax = Eigen::VectorXd::Zero(taskSpace_);
    increasingBellShapeParameter_.xmin = Eigen::VectorXd::Zero(taskSpace_);
    AgreaterThan_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);
    AlessThan_ = Eigen::MatrixXd::Zero(taskSpace_, taskSpace_);
}

ReactiveTask::~ReactiveTask() {}

void ReactiveTask::CheckInitialization() noexcept(false)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException e;
        std::string how = "[ReactiveTask] Not initialized taskParameter struct, use TaskParameter() for task " + ID_;
        e.SetHow(how);
        throw(e);
    }
    if (taskType_ == TaskType::Inequality) {
        if (!isLessThanParamsInizialized_ || !isGreaterThanParamsInizialized_) {
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
    if (taskOption_ == tpik::TaskOption::UseErrorNorm) {
        if (x_.norm() == 0.0) {
            J_.resize(1, dof_);
            J_.setZero();
        } else {
            Eigen::MatrixXd Jtmp = J_;
            J_.resize(1, dof_);
            J_ = (x_.transpose() / x_.norm()) * Jtmp;
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

    const libconfig::Setting& task = tasks.lookup(ID_);

    taskParameter_.ConfigureFromFile(task);
    initializedTaskParameter_ = true;

    int tmpType;
    ctb::SetParam(task, tmpType, "type");
    taskType_ = static_cast<tpik::TaskType>(tmpType);

    if (taskType_ == TaskType::Inequality) {

        if (task.lookup("greaterThanParams")) {
            const libconfig::Setting& decbellShapedParam = task.lookup("greaterThanParams");
            decreasingBellShapeParameter_.ConfigureFromFile(decbellShapedParam);
            isGreaterThanParamsInizialized_ = true;
        }

        if (task.lookup("lessThanParams")) {
            const libconfig::Setting& incbellShapedParam = task.lookup("lessThanParams");
            increasingBellShapeParameter_.ConfigureFromFile(incbellShapedParam);
            isLessThanParamsInizialized_ = true;
        }
    }
}
}
