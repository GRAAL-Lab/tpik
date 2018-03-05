#include <iostream>
#include "InequalityTask.h"

namespace tpik {

InequalityTask::InequalityTask(const std::string ID):Task(ID),minBound_(0),maxBound_(0){}

InequalityTask::~InequalityTask(){
}
void InequalityTask::SetMinBound(double minBound) {
	minBound_ = minBound;

}

void InequalityTask::SetMaxBound(double maxBound) {
	maxBound_ = maxBound;

}

void InequalityTask::SetTaskParameter(TaskParameter taskParameter) {
	taskParameter_ = taskParameter;
}

TaskParameter InequalityTask::GetTaskParameter() {
	return taskParameter_;
}

void InequalityTask::SetBellShapedParameter(BellShapedParameter bellShapedParameter){
	bellShapedParameter_=bellShapedParameter;
}

BellShapedParameter InequalityTask::GetBellShapedParameter(){
	return bellShapedParameter_;
}

}
