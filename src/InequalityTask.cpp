#include <iostream>
#include "tpik/InequalityTask.h"
#include "rml/RML.h"
#include "tpik/TPIKExceptions.h"

namespace tpik
{

InequalityTask::InequalityTask(const std::string ID, int taskSpace, int DoF, bool bellShapeDecreasingUsed,
		bool BellShapedIncreasingUsed) :
		Task(ID, taskSpace, DoF), initializedDecreasingBellShapeParameter_(false), initializedIncreasingBellShapeParameter_(
				false), initializedTaskParameter_(false), bellShapeDecreasingUsed_(bellShapeDecreasingUsed), bellShapeIncreasingUsed_(bellShapeIncreasingUsed_)
{

}

InequalityTask::~InequalityTask()
{
}

void InequalityTask::SetTaskParameter(TaskParameter taskParameters)
{
	taskParameter_ = taskParameters;
	isActive_ = taskParameter_.taskEnable;
	initializedTaskParameter_ = true;

}

const TaskParameter& InequalityTask::GetTaskParameter()
{
	return taskParameter_;
}

void InequalityTask::SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShaped)
{
	increasingBellShape_ = increasingBellShaped;
	initializedIncreasingBellShapeParameter_ = true;

}
void InequalityTask::SetDecreasingBellShapedParameter(BellShapedParameter decreasingBellShaped)
{
	decreasingBellShape_ = decreasingBellShaped;
	initializedDecreasingBellShapeParameter_ = true;
}

const BellShapedParameter& InequalityTask::GetIncreasingBellShapedParameter()
{
	return increasingBellShape_;
}

const BellShapedParameter& InequalityTask::GetDecreasingBellShapedParameter()
{
	return decreasingBellShape_;
}

void InequalityTask::CheckInitialization() throw (std::exception)
{
	if (!initializedTaskParameter_)
	{
		TaskParameterNotInitializedException taskParameterException;
		taskParameterException.SetID(ID_);
		throw(taskParameterException);
	}
	if (!initializedIncreasingBellShapeParameter_ & bellShapeIncreasingUsed_)
	{
		BellShapedIncreasingNotInitializedException bellShapeIncreasingNotInitilized;
		bellShapeIncreasingNotInitilized.SetID(ID_);
		throw(bellShapeIncreasingNotInitilized);
	}
	if (!initializedDecreasingBellShapeParameter_ & bellShapeDecreasingUsed_)
	{
		BellShapedDecreasingNotInitializedException bellShapedDecreasingNotInitialized;
		bellShapedDecreasingNotInitialized.SetID(ID_);
		throw(bellShapedDecreasingNotInitialized);
	}

}

bool InequalityTask::GetBellShapeIncreasingUsed()
{
	return bellShapeIncreasingUsed_;
}

bool InequalityTask::GetBellShapeDecreasingUsed()
{
	return bellShapeDecreasingUsed_;
}

void InequalityTask::SaturateReference()
{
	rml::SaturateVector(taskSpace_, taskParameter_.saturation, x_dot_);
}

}
