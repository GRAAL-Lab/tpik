#include <iostream>
#include "EqualityTask.h"

namespace tpik {

EqualityTask::EqualityTask(const std::string ID, int TaskSpace, int DoF) :
		Task(ID, TaskSpace, DoF) {
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
