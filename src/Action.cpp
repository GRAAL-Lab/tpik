#include "tpik/Action.h"

namespace tpik
{

Action::Action()
{

}

Action::~Action()
{

}

void Action::SetID(std::string ID)
{
	ID_ = ID;
}

bool Action::FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel)
{
	for (auto& priorityLevelHierarhcy : priorityLevels_) {
		if (priorityLevelHierarhcy->GetID() == (priorityLevel->GetID())) {
			return true;
		}
	}
	return false;
}

void Action::AddPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel)
{
	priorityLevels_.push_back(priorityLevel);
}

const Hierarchy& Action::GetPriorityLevels() const
{
	return priorityLevels_;
}

std::string Action::GetID()
{
	return ID_;
}

}
