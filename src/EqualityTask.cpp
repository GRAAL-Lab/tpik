#include "tpik/EqualityTask.h"
#include "tpik/TPIKExceptions.h"
#include "rml/RML.h"

namespace tpik
{

EqualityTask::EqualityTask(const std::string ID, int TaskSpace, int DoF) :
		Task(ID, TaskSpace, DoF)
{
	initializedTaskParameter_ = false;
	UpdateInternalActivationFunction();
}

EqualityTask::~EqualityTask()
{
}

void EqualityTask::SetTaskParameter(TaskParameter taskParameter)
{
	taskParameter_ = taskParameter;
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

void EqualityTask::CheckInitialization() throw (std::exception)
{
	if (!initializedTaskParameter_) {
		std::cout << "TASK ID " << ID_ << std::endl;
		throw(TaskParameterNotInitializedException());
	}
}

void EqualityTask::UpdateInternalActivationFunction()
{
	Ai_.setIdentity();
}
}
