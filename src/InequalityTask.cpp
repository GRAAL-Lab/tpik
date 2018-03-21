#include <iostream>
#include "tpik/InequalityTask.h"
#include "rml/RML.h"
#include "tpik/TPIKExceptions.h"

namespace tpik
{

InequalityTask::InequalityTask(const std::string ID, int TaskSpace, int DoF) :
		Task(ID, TaskSpace, DoF)
{
	minBound_.resize(taskSpace_);
	maxBound_.resize(taskSpace_);
	initializedMinBound_ = false;
	initializedMaxBound_ = false;
	initializedBellShapeParameters_ = false;
	initializedTaskParameter_ = false;
}

InequalityTask::~InequalityTask()
{
}

void InequalityTask::SetMinBound(Eigen::VectorXd minBound)
{
	minBound_ = minBound;
	initializedMinBound_ = true;

}

void InequalityTask::SetMaxBound(Eigen::VectorXd maxBound)
{
	maxBound_ = maxBound;
	initializedMaxBound_ = true;

}

void InequalityTask::SetTaskParameter(TaskParameter taskParameters)
{
	taskParameter_ = taskParameters;
	initializedTaskParameter_ = true;
}

TaskParameter InequalityTask::GetTaskParameter()
{
	return taskParameter_;
}

void InequalityTask::SetBellShapedParameter(BellShapedParameter bellShapedParameter)
{
	bellShapedParameter_ = bellShapedParameter;
	initializedBellShapeParameters_ = true;
}

BellShapedParameter InequalityTask::GetBellShapedParameter()
{
	return bellShapedParameter_;
}

void InequalityTask::SaturateReference()
{
	rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
}

void InequalityTask::CheckInitialization() throw (std::exception)
{
	if (!initializedTaskParameter_) {
		throw(TaskParameterNotInitializedException());
	}
	if (!initializedMaxBound_ & maxBoundUsed_) {
		throw(MaxBoundNotInitializedException());
	}
	if (!initializedMinBound_ & minBoundUsed_ ) {
		throw(MinBoundNotInitializedException());
	}
	if (!initializedBellShapeParameters_) {
		throw(BellShapeParametersNotInitializedException());
	}
}

}
