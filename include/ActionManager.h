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
	 * @brief Action Manager constructor.
	 * @param[in] hierarchy : unified hierarchy.
	 *  */
	ActionManager(Hierarchy hierarchy);
	/**
	 * @brief Action Manager Default constructor.
	 *  */
	ActionManager();
	/**
	 * @brief Method that adds a tpik::Action to the ActionManager vector of action.
	 * @param[in] action : shared_ptr to the tpik::Action.
	 *  */
	void AddAction(std::shared_ptr<Action> action);
	/**
	 * @brief Method that finds an tpik::Action in the actionManager vector of actions.
	 * @param[in] ID: action ID.
	 * @return shared ptr to the found tpik::Action.
	 *  */
	std::shared_ptr<Action> FindAction(std::string ID);
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
	 * @brief Method which sets the unified hierarchy
	 *  */
	void SetHierarchy(Hierarchy hierarchy);
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


protected:
	std::vector<std::shared_ptr<Action>> actions_;
	Hierarchy hierarchy_;
	std::shared_ptr<Action> currentAction_;
	std::shared_ptr<Action> oldAction_;
	std::chrono::system_clock::time_point time_;

};
}

#endif

