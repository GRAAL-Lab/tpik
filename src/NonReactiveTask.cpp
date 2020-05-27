#include "tpik/NonReactiveTask.h"
#include "rml/RML.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

NonReactiveTask::NonReactiveTask(const std::string ID, int taskSpace, int DoF)
    : Task(ID, taskSpace, DoF)
    , initializedTaskParameter_{ false }
    , taskParameter_{ 0.0, false, 0.0 }
    , saturateRaferenceRateComponentWise_{ false }
{
    x_bar_.Eigen::VectorXd::Zero(taskSpace_);
}

NonReactiveTask::~NonReactiveTask(){};

void NonReactiveTask::CheckInitialization() noexcept(false)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[NonReactiveTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
}

void NonReactiveTask::ConfigFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& tasks = root["tasks"];

    const libconfig::Setting& task = tasks.lookup(ID_);

    taskParameter_.ConfigureFromFile(task);
    initializedTaskParameter_ = true;

    if (task.exists("saturateRaferenceRateComponentWise")) {
        task.lookupValue("saturateRaferenceRateComponentWise", saturateRaferenceRateComponentWise_);
    }
}

void NonReactiveTask::SaturateReferenceRate()
{
    if (saturateRaferenceRateComponentWise_) {
        for (int i = 0; i < taskSpace_; i++) {
            rml::SaturateScalar(taskParameter_.saturation, x_dot_(i));
        }
    } else {
        rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
    }
}

void NonReactiveTask::UpdateInternalActivationFunction() { Ai_.setIdentity(); }

void NonReactiveTask::UpdateReferenceRate() { x_dot_ = x_bar_; }

void NonReactiveTask::UpdateReference() {}
}
