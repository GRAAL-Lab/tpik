#include <iostream>
#include "tpik/EqualityTask.h"
#include "rml/RML.h"

namespace tpik
{

EqualityTask::EqualityTask(const std::string ID, int TaskSpace, int DoF) :
		Task(ID, TaskSpace, DoF)
{
}

EqualityTask::~EqualityTask()
{
}

void EqualityTask::SetTaskParameter(TaskParameter taskParameter)
{
	taskParameter_ = taskParameter;
}

TaskParameter EqualityTask::GetTaskParameter()
{
	return taskParameter_;
}

void EqualityTask::SaturateReference()
{
	rml::SaturateVector(taskSpace_,taskParameter_.saturation,x_dot_);
}
}
