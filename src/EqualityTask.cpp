#include <iostream>
#include "EqualityTask.h"

namespace tpik {

EqualityTask::EqualityTask(const std::string ID) :
		Task(ID) {
}

EqualityTask::~EqualityTask() {
}

void EqualityTask::SetTaskParameter(TaskParameter taskParameter) {
	taskParameter_ = taskParameter;
}

TaskParameter EqualityTask::GetTaskParameter() {
	return taskParameter_;
}

}
