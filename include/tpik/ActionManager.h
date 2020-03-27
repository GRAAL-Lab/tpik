#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include <iostream>
#include <vector>
#include "Action.h"
#include "PriorityLevel.h"
#include <chrono>
#include "TPIKDefines.h"
#include "TPIKExceptions.h"

namespace tpik
{
/**
 * @brief Action Manager Class.
 * @details Implementation of the Action Manager class. Such class offers an API in order to create the tpik::PriorityLevel, create the unified hierarchy,
 * define the tpik::Action.\n
 * The tpik::ActionManager can be used in simulation by setting the related boolean to true (via SetIsSimulation()) and setting the simulationTime
 * (via SetTime()).
 * @note The order in which the tpik::PriorityLevel are created defines their priority in the unified hierarchy.
 */
class ActionManager
{
public:
	/**
	 * @brief Action Manager Default constructor.\n
     * @details The ActionManager is, by default, NOT used in a simulation, hence the time is syncronized with the system clock time.
	 * @note The default current action is an empty action.
	 */
	ActionManager();
	/**
	 * @brief Method creating a new tpik::PriorityLevel in the unified hierarchy.\n
	 * The order in which the tpik::PriorityLevel are created defines their priority in the unified hierarchy.
	 * @param[in] priorityLevelID priority level id.
	 */
    void AddPriorityLevel(const std::string priorityLevelID);
	/**
	 * @brief Method creating a new tpik::PriorityLevel in the unified hierarchy and specifies its rml::RegularizationData.\n
	 * The order in which the tpik::PriorityLevel are created defines their priority in the unified hierarchy.
	 * @param[in] priorityLevelID priority level id.
	 * @param[in] regularizationData rml::RegularizationData of the PriorityLevel.
	 */
    void AddPriorityLevelWithRegularization(const std::string priorityLevelID,
			rml::RegularizationData regularizationData);
	/**
	 * @brief Method adding a tpik::Task to a tpik::PriorityLevel.
	 * @note An exception is thrown if the tpik::PriorityLevel does not exist in the unified hierarchy (Via GetPriorityLevel method).
	 * @param[in] task std::shared_ptr to the tpik::Task.
	 * @param[in] priorityLevelID tpik::PriorityLevel ID.
	 */
	void AddTaskToPriorityLevel(std::shared_ptr<Task> task, const std::string priorityLevelID);
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
	void AddAction(std::string actionID, std::vector<std::string> priorityLevelsID);
	/**
	 * @brief Method that sets the current Action.
	 * @param[in] newAction new current action ID.
	 * @note An exception is thrown if the action set is not present in the action list (via getAction exception).
	 */

    void SetAction(std::string newAction, bool transition);

    /**
     * @brief Method returning the current Action ID.
     * @return current action id
     */
    const std::string GetCurrentAction();
	/**
	 * @brief Method which computes and sets the external activation functions in the unified hierarchy tpik::PriorityLevel.\n
	 * The external activation functions depend on the current action, the past action and the time elapsed since the last change of action.
	 * @note An exception is thrown if the unified hierarchy has not been specified yet.
	 */
    void ComputeActionTransitionActivation() throw (ExceptionWithHow);
	/**
	 * @brief Method which returns the unified hierarchy.
	 * @return Unified Hierarchy.
	 * @note  An exception is thrown if the unified hierarchy has not been specified yet.
	 */
    const Hierarchy& GetHierarchy() const throw (ExceptionWithHow);
	/**
	 * @brief Method which returns a tpik::PriorityLevel.
	 * @param[in] priorityLevelID ID of the tpik::PriorityLevel to find.
	 * @return std::shared_ptr to tpik::PriorityLevel.
	 * @note An exception is thrown if the priority level does not exist in the unified hierarchy.
	 */

    std::shared_ptr<PriorityLevel> GetPriorityLevel(std::string priorityLevelID);
	/**
	 * @brief Method to set whether the actionManager is used in a simulation.
	 * @param[in] isSimulated true if it is simulated, false otherwise.
	 */
	void SetIsSimulation(bool isSimulated);
	/**
	 * @brief Method to set the time if the tpik::ActionManager is used in a simulation.
	 * @param[in] simulationTime currentSimulationTime in ms.
	 */
	void SetTime(long simulationTime);

	/**
	 * @brief Method which returns a tpik::Action.
	 * @param[in] ActionID ID of the Action to find.
	 * @return std::shared_ptr to Action.
	 * @note An exception is thrown if the action set is not present in the action list.
	 */
    std::shared_ptr<Action> GetAction(std::string ActionID) throw (ExceptionWithHow);
	/**
	 * @brief Overload of the cout operator
	 */
	friend std::ostream& operator <<(std::ostream& os, ActionManager const& actionManager)
	{
		std::time_t ttp = std::chrono::system_clock::to_time_t(actionManager.time_);
		return os << "\033[1;37m" << "ActionManager \n" << std::setprecision(4) <<
				"Current Action " << "\033[0m" << *actionManager.currentAction_ << "\033[1;37m"
				<< "OldAction " << "\033[0m" << *actionManager.oldAction_ << "\033[1;37m" << std::endl;
	}
	/**
	 * @brief Method which returns the current time. If the tpik::ActionManager is simulated the simulation time is returned,
	 * else the current system clock is returned.
	 * @return current time.
	 */
    //TODO it is private, only for debug purposes
	std::chrono::system_clock::time_point GetTime();
protected:
    std::unordered_map<std::string, std::shared_ptr<tpik::PriorityLevel>> priorityLevelIDMap_;
	std::vector<std::shared_ptr<Action>> actions_; //!< The action list.
	Hierarchy hierarchy_; //!< The unified hierarchy.
	std::shared_ptr<Action> currentAction_; //!< The current action.
	std::shared_ptr<Action> oldAction_; //!< The previous action.
	std::chrono::system_clock::time_point time_; //!< The current time.
	bool isSimulated_{false}; //!< The boolean stating whether the action manager is used in a simulation.
	std::chrono::system_clock::time_point simulationTime_; //!< The simulation time.
	std::chrono::system_clock::time_point simulationBegin_; //!< The time when the simulation begin.
    bool transitionInBetweenActions_;
};
}

#endif

