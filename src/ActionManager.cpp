#include "tpik/ActionManager.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

ActionManager::ActionManager()
    : isSimulated_(false)
{
    auto defaultAct = std::make_shared<Action>(Action());
    defaultAct->SetID("DEFAULT_ACTION");
    oldAction_ = defaultAct;
    currentAction_ = defaultAct;
    actions_.push_back(oldAction_);
    simulationBegin_ = std::chrono::system_clock::now();
    simulationTime_ = simulationBegin_;
    transitionInBetweenActions_ = true;
}

void ActionManager::AddPriorityLevel(const std::string priorityLevelID)
{
    // hierarchy_.push_back(std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID)));
    priorityLevelIDMap_.insert(std::make_pair(priorityLevelID, std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID))));
}

void ActionManager::AddPriorityLevelWithRegularization(const std::string priorityLevelID, rml::RegularizationData regularizationData)
{
    auto pl = std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID));
    pl->SetRegularizationData(regularizationData);
    priorityLevelIDMap_.insert(std::make_pair(priorityLevelID, pl));
    // hierarchy_.push_back(pl);
}

void ActionManager::AddTaskToPriorityLevel(std::shared_ptr<Task> task, const std::string priorityLevelID)
{
    // std::shared_ptr<PriorityLevel> pl = GetPriorityLevel(priorityLevelID);
    auto pl = priorityLevelIDMap_.at(priorityLevelID);
    pl->AddTask(task);
}

void ActionManager::AddAction(std::string actionID, std::vector<std::string> priorityLevelsID)
{
    auto newAction = std::make_shared<Action>(Action());
    newAction->SetID(actionID);
    for (auto& pl : priorityLevelsID) {
        std::shared_ptr<PriorityLevel> plToAdd = priorityLevelIDMap_.at(pl);
        newAction->AddPriorityLevel(plToAdd);
    }
    actions_.push_back(newAction);
}
void ActionManager::SetUnifiedHierarchy(std::vector<std::string> unifiedHierarchy)
{
    hierarchy_.clear();
    for (auto& id : unifiedHierarchy) {
        hierarchy_.push_back(priorityLevelIDMap_.at(id));
    }
}

void ActionManager::SetAction(std::string newAction, bool transition)

{
    oldAction_ = currentAction_;
    currentAction_ = GetAction(newAction);
    time_ = GetTime();
    transitionInBetweenActions_ = transition;
}
const std::string ActionManager::GetCurrentAction()
{
    return currentAction_->GetID();
}

void ActionManager::ComputeActionTransitionActivation() throw(ExceptionWithHow)
{
    if (hierarchy_.empty()) {
        ActionManagerException nullHierarchyExcpetion;
        std::string how = "No unified heirarcy initialized, to add priorityLevel "
                          "to the hierarhcy use either "
                          "AddPriorityLevelToHierarchyWithRegularization() or"
                          "AddPriorityLevelToHierarchyWithRegularization ";
        nullHierarchyExcpetion.SetHow(how);
        throw(nullHierarchyExcpetion);
    }
    for (auto& priorityLevel : hierarchy_) {
        bool isInOldAction = oldAction_->FindPriorityLevel(priorityLevel);
        bool isInNewAction = currentAction_->FindPriorityLevel(priorityLevel);
        double actionTransitionA_;
        if (isInOldAction && isInNewAction) {

            // The PL is already active :identity.
            actionTransitionA_ = 1.0;
        } else if (!isInOldAction && isInNewAction) {
            // PL must be activated
            if (transitionInBetweenActions_) {
                // The PL must be activated: increasing.

                std::chrono::duration<double, std::milli> diff = GetTime() - time_;
                // In 500 ms the PL is completely active.
                actionTransitionA_ = rml::IncreasingBellShapedFunction(0.00, 500.0, 0, 1, diff.count());
            } else {
                actionTransitionA_ = 1.0;
            }
        } else if (isInOldAction && !isInNewAction) {
            // PL must be deactivated
            if (transitionInBetweenActions_) {
                std::chrono::duration<double, std::milli> diff = GetTime() - time_;
                // The PL must be deactivated: decreasing.
                actionTransitionA_ = rml::DecreasingBellShapedFunction(0.00, 500.0, 0, 1, diff.count());

            } else {
                actionTransitionA_ = 0.0;
            }
        } else {
            actionTransitionA_ = 0.0;
            // The PL is already deactivated:zeros.
        }
        priorityLevel->SetActionTransitionActivation(actionTransitionA_);
    }
}

const Hierarchy& ActionManager::GetHierarchy() const throw(ExceptionWithHow)
{
    if (hierarchy_.empty()) {
        ActionManagerException nullHierarchyExcpetion;
        std::string how = "No unified heirarcy initialized, to add priorityLevel "
                          "to the hierarhcy use either "
                          "AddPriorityLevelToHierarchyWithRegularization() or"
                          "AddPriorityLevelToHierarchyWithRegularization ";
        nullHierarchyExcpetion.SetHow(how);
        throw(nullHierarchyExcpetion);
    }
    return hierarchy_;
}

std::shared_ptr<Action> ActionManager::GetAction(std::string actionID) throw(ExceptionWithHow)
{
    for (auto& act : actions_) {
        if (act->GetID() == actionID) {
            return act;
        }
    }
    ActionManagerException nullAction;
    std::string how = "Asking a not existing action " + actionID;
    nullAction.SetHow(how);
    throw(nullAction);
}

std::shared_ptr<PriorityLevel> ActionManager::GetPriorityLevel(std::string priorityLevelID)
{
    return priorityLevelIDMap_.at(priorityLevelID);
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
        return std::chrono::time_point_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now());
    }
}
} // namespace tpik
