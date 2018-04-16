#include "tpik/ActionManager.h"
#include "tpik/TPIKExceptions.h"

namespace tpik
{

ActionManager::ActionManager():isSimulated_(false)
{
	auto defaultAct = std::make_shared<Action>(Action());
	defaultAct->SetID("DEFAULT_ACTION");
	oldAction_ = defaultAct;
	currentAction_ = defaultAct;
	actions_.push_back(oldAction_);
	simulationBegin_ = std::chrono::system_clock::now();
	simulationTime_ = simulationBegin_;
}

void ActionManager::AddPriorityLevelToHierarchy(const std::string priorityLevelID)
{
	hierarchy_.push_back(std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID)));
}

void ActionManager::AddPriorityLevelToHierarchyWithRegularization(const std::string priorityLevelID,
		rml::RegularizationData regularizationData)
{
	auto pl = std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID));
	pl->SetRegularizationData(regularizationData);
	hierarchy_.push_back(pl);
}

void ActionManager::AddTaskToPriorityLevel(std::shared_ptr<Task> task, const std::string priorityLevelID)
{
	std::shared_ptr<PriorityLevel> pl = GetPriorityLevel(priorityLevelID);
	pl->AddTask(task);
}

void ActionManager::AddAction(std::string actionID, std::vector<std::string> priorityLevelsID)
{
	auto newAction = std::make_shared<Action>(Action());
	newAction->SetID(actionID);
	for (auto& pl : priorityLevelsID) {
		std::shared_ptr<PriorityLevel> plToAdd = GetPriorityLevel(pl);
		newAction->AddPriorityLevel(plToAdd);
	}
	actions_.push_back(newAction);
}

void ActionManager::SetAction(std::string newAction)
{
	oldAction_ = currentAction_;
	currentAction_ = GetAction(newAction);
	time_ = GetTime();
}

void ActionManager::ComputeExternalActivation() throw (std::exception)
{
	if (hierarchy_.empty()) {
		throw(ActionManagerHierarchyException());
	}
	for (auto& priorityLevel : hierarchy_) {
		bool isInOldAction = oldAction_->FindPriorityLevel(priorityLevel);
		bool isInNewAction = currentAction_->FindPriorityLevel(priorityLevel);
		double Ae;
		if (isInOldAction && isInNewAction) {
			// The PL is already active :identity.
			priorityLevel->SetExternalActivationFunction(1.0);
		} else if (!isInOldAction && isInNewAction) {
			// The PL must be activated: increasing.
			std::chrono::duration<double, std::milli> diff = GetTime() - time_;
			// In 500 ms the PL is completely active.
			Ae = rml::IncreasingBellShapedFunction(0.00, 500, 0, 1, diff.count());
			priorityLevel->SetExternalActivationFunction(Ae);
		} else if (isInOldAction && !isInNewAction) {
			std::chrono::duration<double, std::milli> diff = GetTime() - time_;
			//The PL must be deactivated: decreasing.
			Ae = rml::DecreasingBellShapedFunction(0.00, 500, 0, 1, diff.count());
			priorityLevel->SetExternalActivationFunction(Ae);

		} else {
			//The PL is already deactivated:zeros.
			priorityLevel->SetExternalActivationFunction(0.0);
		}
	}

}

const Hierarchy& ActionManager::GetHierarchy() const throw (std::exception)
{
	if (hierarchy_.empty()) {
		throw ActionManagerHierarchyException();
	}
	return hierarchy_;
}

std::shared_ptr<Action> ActionManager::GetAction(std::string actionID) throw (std::exception)
{
	ActionManagerNullActionException nullAction;
	nullAction.SetID(actionID);
	for (auto& act : actions_) {
		if (act->GetID() == actionID) {
			return act;
		}
	}
	throw(nullAction);
}

std::shared_ptr<PriorityLevel> ActionManager::GetPriorityLevel(std::string priorityLevelID) throw (std::exception)
{
	ActionManagerMissingPriorityLevelException missingPL;
	missingPL.SetID(priorityLevelID);

	for (auto& priorityLevel : hierarchy_) {
		if (priorityLevel->GetID() == priorityLevelID) {
			return priorityLevel;
		}
	}
throw(missingPL);

}

void ActionManager::SetIsSimulation(bool isSimulated)
{
	isSimulated_ = isSimulated;
}

void ActionManager::SetTime(long simulationTime)
{
	auto t = std::chrono::milliseconds(simulationTime);
	simulationTime_ = simulationBegin_ + t;
}

std::chrono::system_clock::time_point ActionManager::GetTime()
{

	if (isSimulated_) {
		return simulationTime_;
	} else {
		return std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	}
}
}
