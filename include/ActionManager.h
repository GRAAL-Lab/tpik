#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "Action.h"
#include "PriorityLevel.h"
#include <chrono>
#include "TPIKExceptions.h"
#include "TPIKDefines.h"

namespace tpik {
/**
 * @brief Action Manager Class
 * Implementation of the Action Manager class. Such class, starting from a vector of tpik::Action and a unified hierarhcy can set
 * the current action, computing and setting the priority level external activation function.
 */
class ActionManager {
public:
	/**
	 * @brief Action Manager Default constructor.
	 *  */
	ActionManager();

	/**
	 * @brief Method creating a new PriorityLevel in the hierarchy, the user must create the priorityLevel taking into account their priority
	 * Hence creating the priorityLevel by ordering them wrt their priority
	 * @param[in] plID priority level id.
	 */
	void AddPriorityLevelToHierarchy(const std::string plID);
	/**
	 * @brief Method adding a task to a priorityLevel.
	 * @param[in] task: shared pointer to the task.
	 * @param[in] plID: priorityLevel ID
	 */
	void AddTaskToPriorityLevel(std::shared_ptr<Task> task,const std::string plID);
	/**
	 * @brief Adding an action to the action list
	 * @param[in] ActionID: Action ID.
	 * @param[in] priorityLevels: std::vector containing the IDs of the priorityLevels composing the Action
	 */
	void AddAction(std::string actionID, std::vector<std::string> priorityLevels);

	/**
	 * @brief Method that sets the current Action
	 * @param[in] newAction: new current action ID
	 *  */
	void SetAction(std::string newAction) throw (ActionManagerNullActionException);
	/**
	 * @brief Method which computes and sets the external activation function in the unified hierarchy priority Levels
	 * The external activation functions depend on the current action, the past action and the time elapsed since the last change of action.
	 *  */
	void ComputeExternalActivation() const throw (ActionManagerHierarchyException);
	/**
	 * @brief Method which returns the unified hierarchy.
	 * @return Unified Hierarchy .
	 *  */
	const Hierarchy& GetHierarchy() const throw (ActionManagerHierarchyException);
	/**
	 * @brief Overload of the cout operator
	 *  */
	friend std::ostream& operator <<(std::ostream& os, ActionManager const& actionManager) {
		std::time_t ttp = std::chrono::system_clock::to_time_t(actionManager.time_);
		return os << "\033[1;37m" << "ActionManager \n" << std::setprecision(2)
				<< "Current Action " << "\033[0m"<< *actionManager.currentAction_ << "\033[1;37m"
				<< "OldAction " << "\033[0m" << *actionManager.oldAction_ << "\033[1;37m"
				<< "Time Elapsed " << "\033[0m" << std::put_time(std::localtime(&ttp), "%F %T");
	}
private:
	/**
	 * @brief Method that finds an tpik::Action in the actionManager vector of actions.
	 * @param[in] ID: action ID.
	 * @return shared ptr to the found tpik::Action.
	 *  */
	std::shared_ptr<Action> FindAction(std::string ID);
	/**
	 * @brief Method which finds a priority level in the unified hierarchy.
	 * @param[in] priorityLevelID: ID of the priority level to find.
	 * @return shared pointer to the found priorityLevel.
	 */
	std::shared_ptr<PriorityLevel> FindPriorityLevelInHierarchy(std::string priorityLevelID);

protected:
	std::vector<std::shared_ptr<Action>> actions_;
	Hierarchy hierarchy_;
	std::shared_ptr<Action> currentAction_;
	std::shared_ptr<Action> oldAction_;
	std::chrono::system_clock::time_point time_;

};
}

#endif

