#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <vector>
#include <memory>
#include "PriorityLevel.h"

namespace tpik {
typedef std::vector<std::shared_ptr<tpik::PriorityLevel> > Hierarchy;
/**
 * @brief Action class.
 * Implementation of the Action class, composed by an ID and a vector of shared_ptr to priorityLevels which identify the
 * action hierarchy.
 */

class Action {
public:
	/**
	 * @brief Method setting the Action ID.
	*/
	void SetID(std::string ID);
	/**
	 * @brief Method checking whether the input priorityLevel is present in the action.
	 * @param[in] priorityLevel: shared_ptr to the tpik::PriorityLevel.
	 * @return true if priorityLevel is in the action hierarchy, false otherwise.
	 */
	bool FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	/**
	 * @brief Method adding the input priority level to the action.
	 * @param[in] priorityLevel: shared_ptr to the tpik::PriorityLevel to be added.
	 */
	void AddPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	/**
	 * Method returning the action hierarchy;
	 * @return Action Hierarchy.
	 */
	const Hierarchy GetPriorityLevels() const;
	/**
	 * @brief Method returning the action ID.
	 * @return Action ID.
	 */
	std::string GetID();
	/**
	 * @brief Overloading of the cout operator.
	 */
	friend std::ostream& operator <<(std::ostream& os, Action const& action) {
		return os << "\033[1;37m" << "Action ID " << action.ID_ << "\n" << std::setprecision(2);
	}
	;
private:
	Hierarchy priorityLevels_;
	std::string ID_;
};
}
#endif
