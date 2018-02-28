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

void ActionManager::AddPriorityLevelToHierarchy(const std::string plID){
	hierarchy_.push_back(std::make_shared<PriorityLevel> (PriorityLevel(plID)));
}

void ActionManager::AddTaskToPriorityLevel(std::shared_ptr<Task> task,const std::string plID){
	std::shared_ptr<PriorityLevel> pl=FindPriorityLevelInHierarchy(plID);
	pl->AddTask(task);
}

void ActionManager::AddAction(std::string actionID, std::vector<std::string> priorityLevels){
	auto newAction= std::make_shared<Action> (Action());
	newAction->SetID(actionID);
	for(auto& pl:priorityLevels){
		newAction->AddPriorityLevel(FindPriorityLevelInHierarchy(pl));
	}
	actions_.push_back(newAction);
}

std::shared_ptr<Action> ActionManager::FindAction(std::string ID) {
	for (auto& act : actions_) {
		if (act->GetID() == ID) {
			return act;
		}
	}
	return nullptr;
}

std::shared_ptr<PriorityLevel> ActionManager::FindPriorityLevelInHierarchy(std::string priorityLevelID){

	for(auto& priorityLevel:hierarchy_){
		if(priorityLevel->GetID()==priorityLevelID){
			return priorityLevel;
		}
	}
	return nullptr;

}
void ActionManager::SetAction(std::string newAction) throw (ActionManagerNullActionException) {
	oldAction_ = currentAction_;
	currentAction_ = FindAction(newAction);
	if (currentAction_ == nullptr) {
		throw(ActionManagerNullActionException());
	}
	time_ = std::chrono::system_clock::now();
}

void ActionManager::ComputeExternalActivation() const throw (ActionManagerHierarchyException) {
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


}
