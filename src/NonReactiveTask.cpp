#include "tpik/NonReactiveTask.h"
#include "rml/RML.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

NonReactiveTask::NonReactiveTask(const std::string ID, int taskSpace, int DoF)
    : Task(ID, taskSpace, DoF)
    , initializedTaskParameter_{ false }
    , taskParameter_{ 0.0, false, 0.0 }
    , isReferenceSet_(false)
{
    xReference_.setZero(taskSpace_);
}

NonReactiveTask::~NonReactiveTask(){};

void NonReactiveTask::CheckInitialization() throw(ExceptionWithHow)
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

    for (int i = 0; i < tasks.getLength(); ++i) {

        auto& task = tasks[i];

        std::string name;
        ctb::SetParam(task, name, "name");

        if (ID_ == name) {

            taskParameter_.ConfigureFromFile(task);
        }
    }
    initializedTaskParameter_ = true;
}
void NonReactiveTask::SaturateReference() { rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_); }

void NonReactiveTask::SaturateReferenceComponentWise()
{
    for (int i = 0; i < taskSpace_; i++) {

        rml::SaturateScalar(taskParameter_.saturation, x_dot_(i));
    }
}

void NonReactiveTask::UpdateInternalActivationFunction() { Ai_.setIdentity(); }

void NonReactiveTask::UpdateReference() { x_dot_ = xReference_; }
}
