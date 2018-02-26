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
class ActionManager {
public:
	/**
	 * @brief Action Manager constructor .
	 * @parma[in] hierarchy: Unified Hierarchy
	 *  */
	ActionManager(Hierarchy hierarchy);
	/**
	 * @brief Action Manager Default constructor .
	 *  */
	ActionManager();
	/**
	 * @brief Function to add an Action.
	 * @input[in] action: shared_ptr to the action
	 *  */
	void AddAction(std::shared_ptr<Action> action);
	/**
	 * @brief Function that finds an action in the actionManager vector of actions.
	 * @input[in] ID: action ID
	 * @
	 *  */
	std::shared_ptr<Action> FindAction(std::string ID);
	/**
	 * @brief Set the current Action
	 * @input[in] ID: newAction: new current action ID
	 *  */
	void SetAction(std::string newAction)
			throw (ActionManagerNullActionException);
	/**
	 * @brief Function which computes and set the external activation function of the unified hierarhcy priority Level
	 * The external activation functions depend on the current action, the past action and the time elapsed in between.
	 *  */
	void ComputeExternalActivation() const
			throw (ActionManagerHierarchyException);
	/**
	 * @brief Function which returns the unified hierarchy
	 *  */
	const Hierarchy& GetHierarchy() const
			throw (ActionManagerHierarchyException);
	/**
	 * @brief Function which sets the unified hierarchy
	 *  */
	void SetHierarchy(Hierarchy hierarchy);
	/**
	 * @brief Overload of the cout operator
	 *  */
	friend std::ostream& operator <<(std::ostream& os,
			ActionManager const& actionManager) {
		std::time_t ttp = std::chrono::system_clock::to_time_t(
				actionManager.time_);

		return os << "\033[1;37m" << "ActionManager \n" << std::setprecision(2)
				<< "Current Action " << "\033[0m"
				<< *actionManager.currentAction_ << "\033[1;37m" << "OldAction "
				<< "\033[0m" << *actionManager.oldAction_ << "\033[1;37m"
				<< "Time Elapsed " << "\033[0m"
				<< std::put_time(std::localtime(&ttp), "%F %T");
	}
	;

protected:
	std::vector<std::shared_ptr<Action>> actions_;
	Hierarchy hierarchy_;
	std::shared_ptr<Action> currentAction_;
	std::shared_ptr<Action> oldAction_;
	std::chrono::system_clock::time_point time_;

};
}

#endif

