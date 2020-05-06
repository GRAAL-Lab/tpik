#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include "Action.h"
#include "PriorityLevel.h"
#include "TPIKDefines.h"
#include "TPIKExceptions.h"
#include <chrono>
#include <iostream>
#include <vector>

namespace tpik {
/**
 * @brief Action Manager Class.
 * @details Implementation of the Action Manager class. Such class offers an API in order to create the tpik::PriorityLevel, create the unified hierarchy,
 * define the tpik::Action.
 * The tpik::ActionManager can be used in simulation by setting the related boolean to true (via SetIsSimulation()) and setting the simulationTime
 * (via SetTime()).
 * @note The order in which the tpik::PriorityLevel are created defines their priority in the unified hierarchy.
 */
class ActionManager {
public:
    /**
     * @brief Action Manager Default constructor.
     * @details The ActionManager is, by default, NOT used in a simulation, hence the time is syncronized with the system clock time.
	 * @note The default current action is an empty action.
	 */
    ActionManager();
    /*
    * @brief Action Manager Default deconstructor.
    */
    ~ActionManager() {}
    /*
     * @brief Method creating a new tpik::PriorityLevel in the unified hierarchy.
	 * The order in which the tpik::PriorityLevel are created defines their priority in the unified hierarchy.
	 * @param[in] priorityLevelID priority level id.
	 */
    void AddPriorityLevel(const std::string priorityLevelID);
    /**
     * @brief Method creating a new tpik::PriorityLevel in the unified hierarchy and specifies its rml::RegularizationData.
	 * The order in which the tpik::PriorityLevel are created defines their priority in the unified hierarchy.
	 * @param[in] priorityLevelID priority level id.
	 * @param[in] regularizationData rml::RegularizationData of the PriorityLevel.
	 */
    void AddPriorityLevelWithRegularization(const std::string priorityLevelID, const rml::RegularizationData regularizationData);
    /**
	 * @brief Method adding a tpik::Task to a tpik::PriorityLevel.
	 * @note An exception is thrown if the tpik::PriorityLevel does not exist in the unified hierarchy (Via GetPriorityLevel method).
	 * @param[in] task std::shared_ptr to the tpik::Task.
	 * @param[in] priorityLevelID tpik::PriorityLevel ID.
	 */
    void AddTaskToPriorityLevel(const std::shared_ptr<Task> task, const std::string priorityLevelID);
    /**
     * @brief Method setting the unified hierarhcy
     * @param unifiedHierarchy pl id ordered by priority
     */
    void SetUnifiedHierarchy(std::vector<std::string> unifiedHierarchy);
    /**
	 * @brief Method adding a tpik::Action to the action list.
	 * @param[in] actionID Action ID.
	 * @param[in] priorityLevelsID std::vector containing the IDs of the tpik::PriorityLevel composing the tpik::Action.
	 */
    void AddAction(const std::string actionID, const std::vector<std::string> priorityLevelsID);
    /**
	 * @brief Method that sets the current Action.
	 * @param[in] newAction new current action ID.
	 * @note An exception is thrown if the action set is not present in the action list (via getAction exception).
	 */

    void SetAction(const std::string newAction, bool transition);

    /**
     * @brief Method returning the current Action ID.
     * @return current action id
     */
    auto CurrentActionID() const -> const std::string& { return currentAction_->ID(); }
    /**
     * @brief Method which computes and sets the external activation functions in the unified hierarchy tpik::PriorityLevel.
	 * The external activation functions depend on the current action, the past action and the time elapsed since the last change of action.
	 * @note An exception is thrown if the unified hierarchy has not been specified yet.
	 */
    void ComputeActionTransitionActivation() noexcept(false);
    /**
	 * @brief Method which returns the unified hierarchy.
	 * @return Unified Hierarchy.
	 * @note  An exception is thrown if the unified hierarchy has not been specified yet.
	 */
    const Hierarchy& GetHierarchy() const noexcept(false);
    /**
	 * @brief Method which returns a tpik::PriorityLevel.
	 * @param[in] priorityLevelID ID of the tpik::PriorityLevel to find.
	 * @return std::shared_ptr to tpik::PriorityLevel.
	 * @note An exception is thrown if the priority level does not exist in the unified hierarchy.
	 */
    auto GetPriorityLevel(const std::string priorityLevelID) const -> const std::shared_ptr<PriorityLevel>& { return priorityLevelIDMap_.at(priorityLevelID); }
    /**
	 * @brief Method to set whether the actionManager is used in a simulation.
	 * @param[in] isSimulated true if it is simulated, false otherwise.
	 */
    auto IsSimulation(bool isSimulated) -> void { isSimulated_ = isSimulated; }
    /**
	 * @brief Method to set the time if the tpik::ActionManager is used in a simulation.
	 * @param[in] simulationTime currentSimulationTime in ms.
     */
    auto Time(long simulationTime) -> void { simulationTime_ = simulationBegin_ + std::chrono::milliseconds(simulationTime); }
    /**
     * @brief Method which returns the current time. If the tpik::ActionManager is simulated the simulation time is returned,
     * else the current system clock is returned.
     * @return current time.
     */
    //TODO it is private, only for debug purposes
    const std::chrono::system_clock::time_point Time();
    /**
	 * @brief Method which returns a tpik::Action.
	 * @param[in] ActionID ID of the Action to find.
	 * @return std::shared_ptr to Action.
	 * @note An exception is thrown if the action set is not present in the action list.
	 */
    const std::shared_ptr<Action>& GetAction(const std::string& ActionID) noexcept(false);
    /**
	 * @brief Overload of the cout operator
	 */
    friend std::ostream& operator<<(std::ostream& os, ActionManager const& actionManager)
    {
        std::time_t ttp = std::chrono::system_clock::to_time_t(actionManager.time_);
        return os << "\033[1;37m"
                  << "ActionManager \n"
                  << std::setprecision(4) << "Current Action "
                  << "\033[0m" << *actionManager.currentAction_ << "\033[1;37m"
                  << "OldAction "
                  << "\033[0m" << *actionManager.oldAction_ << "\033[1;37m" << std::endl;
    }

protected:
    std::unordered_map<std::string, std::shared_ptr<tpik::PriorityLevel>> priorityLevelIDMap_;
    std::vector<std::shared_ptr<Action>> actions_; //!< The action list.
    Hierarchy hierarchy_; //!< The unified hierarchy.
    std::shared_ptr<Action> currentAction_; //!< The current action.
    std::shared_ptr<Action> oldAction_; //!< The previous action.
    std::chrono::system_clock::time_point time_; //!< The current time.
    bool isSimulated_; //!< The boolean stating whether the action manager is used in a simulation.
    std::chrono::system_clock::time_point simulationTime_; //!< The simulation time.
    std::chrono::system_clock::time_point simulationBegin_; //!< The time when the simulation begin.
    bool transitionInBetweenActions_;
};
}

#endif
