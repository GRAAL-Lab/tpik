#include "ActionManager.h"
#include <iostream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace tpik {
ActionManager::ActionManager() {
	auto defaultAct = std::make_shared<Action>(Action());
	defaultAct->SetID("DEFAULT_ACTION");
	oldAction_ = defaultAct;
	currentAction_ = defaultAct;
	actions_.push_back(oldAction_);
}

void ActionManager::AddPriorityLevelToHierarchy(const std::string priorityLevelID) {
	hierarchy_.push_back(std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID)));
}

void ActionManager::AddPriorityLevelToHierarchyWithSVD(const std::string priorityLevelID,
		rml::SVDParameters svdParameters) {
	auto pl = std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID));
	pl->SetSVDParameters(svdParameters);
	hierarchy_.push_back(pl);
}

void ActionManager::AddTaskToPriorityLevel(std::shared_ptr<Task> task, const std::string priorityLevelID)
		throw (ActionManagerMissingPriorityLevel) {
	std::shared_ptr<PriorityLevel> pl = FindPriorityLevelInHierarchy(priorityLevelID);
	if (pl == nullptr) {
		throw(ActionManagerMissingPriorityLevel());
	}
	pl->AddTask(task);
}

void ActionManager::AddAction(std::string actionID, std::vector<std::string> priorityLevelsID)
		throw (ActionManagerMissingActionPriorityLevel) {
	auto newAction = std::make_shared<Action>(Action());
	newAction->SetID(actionID);
	for (auto& pl : priorityLevelsID) {
		std::shared_ptr<PriorityLevel> plToAdd = FindPriorityLevelInHierarchy(pl);
		if (plToAdd == nullptr) {
			throw(ActionManagerMissingActionPriorityLevel());
		}
		newAction->AddPriorityLevel(plToAdd);
	}
	actions_.push_back(newAction);
}

void ActionManager::SetAction(std::string newAction) throw (ActionManagerNullActionException) {
	oldAction_ = currentAction_;
	currentAction_ = FindAction(newAction);
	if (currentAction_ == nullptr) {
		throw(ActionManagerNullActionException());
	}
	time_ = std::chrono::system_clock::now();
}

void ActionManager::ComputeExternalActivation() throw (ActionManagerHierarchyException) {
	if (hierarchy_.empty()) {
		throw(ActionManagerHierarchyException());
	}
	for (auto& priorityLevel : hierarchy_) {
		bool isInOldAction = oldAction_->FindPriorityLevel(priorityLevel);
		bool isInNewAction = currentAction_->FindPriorityLevel(priorityLevel);
		double Ae;
		if (isInOldAction && isInNewAction) {
			// The PL is already active :identity
			priorityLevel->SetExternalActivationFunction(1.0);
		} else if (!isInOldAction && isInNewAction) {
			//increasing
			std::chrono::duration<double, std::milli> diff = std::chrono::system_clock::now() - time_;
			std::cout << diff.count() << " DIFF" << std::endl;
			Ae = rml::IncreasingBellShapedFunction(0.00, 500, 0, 1, diff.count());
			priorityLevel->SetExternalActivationFunction(Ae);
			std::cout << Ae << " AE" << std::endl;
		} else if (isInOldAction && !isInNewAction) {
			std::chrono::duration<double, std::milli> diff = std::chrono::system_clock::now() - time_;
			//The PL must be deactivated: decreasing
			std::cout << diff.count() << " DIFF" << std::endl;
			Ae = rml::DecreasingBellShapedFunction(0.00, 500, 0, 1, diff.count());
			priorityLevel->SetExternalActivationFunction(Ae);
			std::cout << Ae << " AE" << std::endl;

		} else {
			//The PL is already deactivated:zeros
			priorityLevel->SetExternalActivationFunction(0.0);
		}
	}

}

const Hierarchy& ActionManager::GetHierarchy() const throw (ActionManagerHierarchyException) {
	if (hierarchy_.empty()) {
		throw ActionManagerHierarchyException();
	}
	return hierarchy_;
}

std::shared_ptr<Action> ActionManager::FindAction(std::string ActionID) {
	for (auto& act : actions_) {
		if (act->GetID() == ActionID) {
			return act;
		}
	}
	return nullptr;
}

std::shared_ptr<PriorityLevel> ActionManager::FindPriorityLevelInHierarchy(std::string priorityLevelID) {

	for (auto& priorityLevel : hierarchy_) {
		if (priorityLevel->GetID() == priorityLevelID) {
			return priorityLevel;
		}
	}
	return nullptr;

}

}
