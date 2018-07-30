#include <iostream>
#include "tpik/InequalityTask.h"
#include "rml/RML.h"
#include "tpik/TPIKExceptions.h"

namespace tpik
{

InequalityTask::InequalityTask(const std::string ID, int taskSpace, int DoF, bool bellShapeDecreasingUsed,
		bool bellShapedIncreasingUsed) :
		Task(ID, taskSpace, DoF), initializedDecreasingBellShapeParameter_(false), initializedIncreasingBellShapeParameter_(
				false), initializedTaskParameter_(false), bellShapeDecreasingUsed_(bellShapeDecreasingUsed), bellShapeIncreasingUsed_(bellShapedIncreasingUsed)
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

void InequalityTask::CheckInitialization() throw (ExceptionWithHow)
{
	if (!initializedTaskParameter_)
	{
        NotInitialziedTaskParameterException notInitializedTaskParameter;
        std::string how = "[InequalityTask] Not initialized taskParameter struct, use SetTaskParameter() for task "+ ID_ ;
        notInitializedTaskParameter.SetHow(how);
        throw(notInitializedTaskParameter);
	}
	if (!initializedIncreasingBellShapeParameter_ & bellShapeIncreasingUsed_)
	{
        NotInitialziedTaskParameterException notInitializedBellShapeIncreasing;
        std::string how = "[InequalityTask] Not initialized incresingBellShape struct, use SetIncreasingBellShapedParameter() for task "+ ID_ ;
         notInitializedBellShapeIncreasing.SetHow(how);
        throw( notInitializedBellShapeIncreasing);
	}
    if(increasingBellShape_.xmax.size()!=taskSpace_ && bellShapeIncreasingUsed_){
        NotInitialziedTaskParameterException wrongBellShapeIcreasingSize;
        std::string how = "[InequalityTask] Wrong size incresingBellShape struct, task space = " + std::to_string(taskSpace_)+" use SetIncreasingBellShapedParameter() for task "+ ID_ ;
         wrongBellShapeIcreasingSize.SetHow(how);
        throw( wrongBellShapeIcreasingSize);
    }
	if (!initializedDecreasingBellShapeParameter_ & bellShapeDecreasingUsed_)
	{
        NotInitialziedTaskParameterException notInitializedBellShapeDecreasing;
        std::string how = "[InequalityTask] Not initialized decreasingBellShape struct, use SetDecreasingBellShapedParameter() for task "+ ID_ ;
         notInitializedBellShapeDecreasing.SetHow(how);
        throw( notInitializedBellShapeDecreasing);
	}

    if(decreasingBellShape_.xmax.size()!=taskSpace_ && bellShapeDecreasingUsed_){
        NotInitialziedTaskParameterException wrongBellShapeDecreasingSize;
        std::string how = "[InequalityTask] Wrong size decreasingBellShape struct, task space = " + std::to_string(taskSpace_)+" use SetDecreasingBellShapedParameter() for task "+ ID_ ;
         wrongBellShapeDecreasingSize.SetHow(how);
        throw( wrongBellShapeDecreasingSize);
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
