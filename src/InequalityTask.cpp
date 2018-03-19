#include <iostream>
#include "tpik/InequalityTask.h"
#include "rml/RML.h"

namespace tpik
{

InequalityTask::InequalityTask(const std::string ID, int TaskSpace, int DoF) :
		Task(ID, TaskSpace, DoF)
{
	minBound_.resize(taskSpace_);
	maxBound_.resize(taskSpace_);
}

InequalityTask::~InequalityTask()
{
}

void InequalityTask::SetMinBound(Eigen::VectorXd minBound)
{
	minBound_ = minBound;

}

void InequalityTask::SetMaxBound(Eigen::VectorXd maxBound)
{
	maxBound_ = maxBound;

}

void InequalityTask::SetTaskParameter(TaskParameter taskParameters)
{
	taskParameter_ = taskParameters;
}

TaskParameter InequalityTask::GetTaskParameter()
{
	return taskParameter_;
}

void InequalityTask::SetBellShapedParameter(BellShapedParameter bellShapedParameter)
{
	bellShapedParameter_ = bellShapedParameter;
}

BellShapedParameter InequalityTask::GetBellShapedParameter()
{
	return bellShapedParameter_;
}

void InequalityTask::SaturateReference()
{
	rml::SaturateVector(taskSpace_,taskParameter_.saturation,x_dot_);
}

}
