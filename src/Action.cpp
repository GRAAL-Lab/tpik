#include "tpik/Action.h"

namespace tpik {

Action::Action() {}

Action::~Action() {}

bool Action::FindPriorityLevel(const std::shared_ptr<PriorityLevel>& priorityLevel)
{
    for (auto& priorityLevelHierarhcy : priorityLevels_) {
        if (priorityLevelHierarhcy->ID() == (priorityLevel->ID())) {
            return true;
        }
    }
    return false;
}

void Action::AddPriorityLevel(const std::shared_ptr<PriorityLevel> priorityLevel)
{
    priorityLevels_.push_back(priorityLevel);
}
}
