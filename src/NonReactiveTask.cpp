#include "tpik/NonReactiveTask.h"
#include "rml/RML.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

NonReactiveTask::NonReactiveTask(const std::string ID, int taskSpace, int DoF)
    : Task(ID, taskSpace, DoF)
    , initializedTaskParameter_ { false }
    , taskParameter_ { 0.0, false, 0.0 }
    , saturateRaferenceRateComponentWise_ { false }
{
    x_dot_bar_ = Eigen::VectorXd::Zero(taskSpace_);
}

NonReactiveTask::~NonReactiveTask() {}

void NonReactiveTask::CheckInitialization() noexcept(false)
{
    if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[NonReactiveTask] Not initialized taskParameter struct, use TaskParameter() for task " + ID_;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
    }
}

bool NonReactiveTask::ConfigFromFile(libconfig::Config& confObj) noexcept(false)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& tasks = root["tasks"];

    //Check if the task name exist in the conf file.
    assert(tasks.exists(ID_));

    const libconfig::Setting& task = tasks.lookup(ID_);

    if (taskParameter_.ConfigureFromFile(task)) {
        initializedTaskParameter_ = true;
    }

    if (task.exists("saturateRaferenceRateComponentWise")) {
        if (ctb::SetParam(task, saturateRaferenceRateComponentWise_, "saturateRaferenceRateComponentWise"))
            return false;
    }

    return true;
}

void NonReactiveTask::SaturateReferenceRate()
{
    if (saturateRaferenceRateComponentWise_) {
        for (int i = 0; i < taskSpace_; i++) {
            rml::SaturateScalar(taskParameter_.saturation, x_dot_bar_(i));
        }
    } else {
        rml::SaturateVector(taskParameter_.saturation, x_dot_bar_);
    }
}

void NonReactiveTask::UpdateInternalActivationFunction() { Ai_.setIdentity(); }

void NonReactiveTask::UpdateReferenceRate() { }

}
