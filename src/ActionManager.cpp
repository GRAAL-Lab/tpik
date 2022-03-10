#include "tpik/ActionManager.h"
#include "tpik/TPIKExceptions.h"

namespace tpik {

ActionManager::ActionManager()
    : isSimulated_ { false }
{
    auto defaultAct = std::make_shared<Action>(Action());
    defaultAct->ID() = "DEFAULT_ACTION";
    oldAction_ = defaultAct;
    currentAction_ = defaultAct;
    actions_.push_back(oldAction_);
    simulationBegin_ = std::chrono::system_clock::now();
    simulationTime_ = simulationBegin_;
    transitionInBetweenActions_ = true;
    SetTransitionDuration(500.0);
}

void ActionManager::AddPriorityLevel(const std::string priorityLevelID)
{
    priorityLevelIDMap_.insert(std::make_pair(priorityLevelID, std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID))));
}

void ActionManager::AddPriorityLevelWithRegularization(const std::string priorityLevelID, const rml::RegularizationData regularizationData)
{
    auto pl = std::make_shared<PriorityLevel>(PriorityLevel(priorityLevelID));
    pl->RegularizationData() = regularizationData;
    priorityLevelIDMap_.insert(std::make_pair(priorityLevelID, pl));
}

void ActionManager::AddTaskToPriorityLevel(const std::shared_ptr<Task> task, const std::string priorityLevelID)
{
    auto pl = priorityLevelIDMap_.at(priorityLevelID);
    pl->AddTask(task);

    // While filling the priority levels we populate the map containing the list
    // of the map that will contain the information about the presence of a task
    // in the current action.
    taskInCurrentActionMap_.insert(std::pair<std::string, bool>(task->ID(), false));
}

void ActionManager::AddAction(const std::string actionID, const std::vector<std::string> priorityLevelsID)
{
    auto newAction = std::make_shared<Action>(Action());
    newAction->ID() = std::move(actionID);
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

void ActionManager::SetAction(const std::string newAction, bool transition)
{
    oldAction_ = currentAction_;

    try {
        currentAction_ = GetAction(newAction);
    } catch (tpik::ActionManagerException &e) {
        std::cerr << e.how() << std::endl;
    }

    time_ = Time();
    transitionInBetweenActions_ = transition;


    // Set to false all the taskInCurrentActionMap_
    for (auto &inCurrentAction : taskInCurrentActionMap_)
    {
        inCurrentAction.second = false;
    }

    // Set to true the inCurrentAction variable only for the tasks in the current action
    tpik::Hierarchy hierarchy = currentAction_->PriorityLevels();
    for (auto& priorityLevel : hierarchy) {
        std::vector<std::shared_ptr<tpik::Task>> tasks = priorityLevel->Level();
        for(auto& task : tasks){
            taskInCurrentActionMap_.at(task->ID()) = true;
        }
    }

}

void ActionManager::ComputeActionTransitionActivation() noexcept(false)
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
                std::chrono::duration<double, std::milli> diff = Time() - time_;
                // In 500 ms the PL is completely active.
                actionTransitionA_ = rml::IncreasingBellShapedFunction(0.00, transitionDurationMs_, 0, 1, diff.count());
            } else {
                actionTransitionA_ = 1.0;
            }
        } else if (isInOldAction && !isInNewAction) {
            // PL must be deactivated
            if (transitionInBetweenActions_) {
                std::chrono::duration<double, std::milli> diff = Time() - time_;
                // The PL must be deactivated: decreasing.
                actionTransitionA_ = rml::DecreasingBellShapedFunction(0.00, transitionDurationMs_, 0, 1, diff.count());

            } else {
                actionTransitionA_ = 0.0;
            }
        } else {
            actionTransitionA_ = 0.0;
            // The PL is already deactivated:zeros.
        }
        priorityLevel->ActionTransitionActivation(actionTransitionA_);
    }
}

const Hierarchy& ActionManager::GetHierarchy() const noexcept(false)
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

bool ActionManager::IsTaskInCurrentAction(const std::string &task_id)
{
    return taskInCurrentActionMap_.at(task_id);
}

const std::shared_ptr<Action>& ActionManager::GetAction(const std::string& actionID) noexcept(false)
{
    for (auto& act : actions_) {
        if (act->ID() == actionID) {
            return act;
        }
    }
    ActionManagerException nullAction;
    std::string how = "Asking a non-existing action " + actionID;
    nullAction.SetHow(how);
    throw(nullAction);
}

const std::chrono::system_clock::time_point ActionManager::Time()
{
    if (isSimulated_) {
        return simulationTime_;
    } else {
        return std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    }
}
} // namespace tpik
