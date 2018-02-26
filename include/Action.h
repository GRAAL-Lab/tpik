#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "PriorityLevel.h"

namespace tpik {
typedef std::vector<std::shared_ptr<tpik::PriorityLevel> > Hierarchy;
class Action {
public:
	/**
	 * @brief Function setting the Action ID.
	 *  */
	void SetID(std::string ID);
	/**
	 * @brief Function checking whether the input priorityLevel is present in the action.
	 * @param[in] shared_ptr to the PriorityLevel
	 *  */
	bool FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	/**
	 * @brief Function adding the input priority level to the action.
	 * @param[in] shared_ptr to the PriorityLevel to be added
	 *  */
	void AddPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	/**
	 * @brief Function returning the action priorityLevels.
	 *  */
	const Hierarchy GetPriorityLevels() const;
	/**
	 * @brief Function returning the action ID.
	 *  */
	std::string GetID();
	/**
	 * @brief Overloading of the cout operator.
	 *  */
	friend std::ostream& operator <<(std::ostream& os, Action const& action) {
		return os << "\033[1;37m" << "Action ID " << action.ID_ << "\n"
				<< std::setprecision(2);
	}
	;
private:
	Hierarchy priorityLevels_;
	std::string ID_;
};
}
#endif
