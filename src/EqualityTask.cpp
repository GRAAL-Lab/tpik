#include "tpik/EqualityTask.h"
#include "tpik/TPIKExceptions.h"
#include "rml/RML.h"

namespace tpik
{

EqualityTask::EqualityTask(const std::string ID, int taskSpace, int DoF) :
		Task(ID, taskSpace, DoF), initializedTaskParameter_{false}
{
	UpdateInternalActivationFunction();
}

EqualityTask::~EqualityTask()
{
}

void EqualityTask::SetTaskParameter(TaskParameter taskParameter)
{
	taskParameter_ = taskParameter;
	isActive_ = taskParameter_.taskEnable;
	initializedTaskParameter_ = true;

}

TaskParameter EqualityTask::GetTaskParameter()
{
	return taskParameter_;
}

void EqualityTask::SaturateReference()
{
	rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
}

void EqualityTask::CheckInitialization() throw (ExceptionWithHow)
{
	if (!initializedTaskParameter_) {
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[EqualityTask] Not initialized taskParameter struct, use SetTaskParameter() for task "+ ID_ ;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
	}
}

void EqualityTask::UpdateInternalActivationFunction()
{
	Ai_.setIdentity();
}
}
