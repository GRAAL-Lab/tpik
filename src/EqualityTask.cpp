#include <iostream>
#include "EqualityTask.h"

namespace tpik {

EqualityTask::EqualityTask(const std::string ID) : Task(ID) {}

void EqualityTask::SetTaskParameter(TaskParameter taskParameter) {
	taskParameter_ = taskParameter;
}

TaskParameter EqualityTask::GetTaskParameter() {
	return taskParameter_;
}
EqualityTask::~EqualityTask(){

}
}
