#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include <iostream>
#include <vector>
#include "Action.h"
#include "PriorityLevel.h"
#include <chrono>
#include "TPIKDefines.h"

namespace tpik
{
/**
 * @brief Action Manager Class.
 * Implementation of the Action Manager class. Such class offers an API in order to create the tpik::PriorityLevels, create the unified hierarchy,
 * define the actions.
 */
class ActionManager
{
public:
	/**
	 * @brief Action Manager Default constructor.
	 */
	ActionManager();
	/**
	 * @brief Method creating a new PriorityLevel in the unified hierarchy.
	 * The order in which the PriorityLevels are created defines their priority in the unified hierarchy.
	 * @param[in] plID: priority level id.
	 */
	void AddPriorityLevelToHierarchy(const std::string priorityLevelID);
	/**
	 * @brief Method creating a new PriorityLevel in the unified hierarchy and specifies its rml::SVDParameters.
	 * The order in which the PriorityLevels are created defines their priority in the unified hierarchy.
	 * @param[in] priorityLevelID: priority level id.
	 * @param[in] svdParameter: rml::SVDParameters of the PriorityLevel.
	 */
	void AddPriorityLevelToHierarchyWithSVD(const std::string priorityLevelID, rml::SVDData svdParameters);
	/**
	 * @brief Method adding a task to a priorityLevel.
	 * An exception is throw if the priority level does not exist in the unified hierarchy (Via GetPriorityLevel method).
	 * @param[in] task: std::shared_ptr to the task.
	 * @param[in] priorityLevelID: priorityLevel ID.
	 */
	void AddTaskToPriorityLevel(std::shared_ptr<Task> task, const std::string priorityLevelID);
	/**
	 * @brief Method adding an action to the action list..
	 * @param[in] ActionID: Action ID.
	 * @param[in] priorityLevelsID: std::vector containing the IDs of the priorityLevels composing the Action.
	 */
	void AddAction(std::string actionID, std::vector<std::string> priorityLevelsID);
	/**
	 * @brief Method that sets the current Action.
	 * An exception is thrown if the action set is not present in the action list (via getAction exception).
	 * @param[in] newAction: new current action ID
	 */
	void SetAction(std::string newAction) ;
	/**
	 * @brief Method which computes and sets the external activation functions in the unified hierarchy priority Levels.
	 * The external activation functions depend on the current action, the past action and the time elapsed since the last change of action.
	 * An exception is thrown if the unified hierarchy has not been specified yet.
	 */
	void ComputeExternalActivation() throw (std::exception);
	/**
	 * @brief Method which returns the unified hierarchy.
	 * An exception is thrown if the unified hierarchy has not been specified yet
	 * @return Unified Hierarchy .
	 */
	const Hierarchy& GetHierarchy() const throw (std::exception);
	/**
	 * @brief Method which returns a priorityLevel.
	 * An exception is throw if the priority level does not exist in the unified hierarchy
	 * @param[in] priorityLevelID: ID of the priority Level.
	 * @return std::shared_ptr to PriorityLevel.
	 */

	std::shared_ptr<PriorityLevel> GetPriorityLevel(std::string priorityLevelID) throw (std::exception);
	/**
	 * @brief Method to set whether the actionManager is used in a simulation.
	 * @param[in] isSimulated: true if it is simulated, false otherwise.
	 */
	void SetIsSimulation(bool isSimulated);
	/**
	 * @brief Method to set whether the time if the actionManager is simulated.
	 * @param[in] simulationTime: currentSimulationTime in ms.
	 */
	void SetTime(long simulationTime);

	/**
	 * @brief Method which returns an Action.
	 * An exception is thrown if the action set is not present in the action list
	 * @param[in] ActionID: ID of the Action.
	 * @return std::shared_ptr to Action.
	 */
	std::shared_ptr<Action> GetAction(std::string ActionID) throw (std::exception);
	/**
	 * @brief Overload of the cout operator
	 */
	friend std::ostream& operator <<(std::ostream& os, ActionManager const& actionManager)
	{
		std::time_t ttp = std::chrono::system_clock::to_time_t(actionManager.time_);
		return os << "\033[1;37m" << "ActionManager \n" << std::setprecision(2)
				<< "Current Action " << "\033[0m" << *actionManager.currentAction_ << "\033[1;37m"
				<< "OldAction " << "\033[0m" << *actionManager.oldAction_ << "\033[1;37m"
				<< "Time Elapsed " << "\033[0m" << std::put_time(std::localtime(&ttp), "%F %T");
	}
	//TODO it is private, only for debug purpose
	std::chrono::system_clock::time_point GetTime();
protected:
	/**
	 * @brief Method which returns the current time. If the action manager is simulated the simulation time is returned,
	 * else the current system clock is returned.
	 * @return current time.
	 */

	std::vector<std::shared_ptr<Action>> actions_;
	Hierarchy hierarchy_;
	std::shared_ptr<Action> currentAction_;
	std::shared_ptr<Action> oldAction_;
	std::chrono::system_clock::time_point  time_;
	bool isSimulated_;
	std::chrono::system_clock::time_point simulationTime_, simulationBegin_;

};
}

#endif

