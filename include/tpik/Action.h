#ifndef __ACTION_H__
#define __ACTION_H__

#include "PriorityLevel.h"
#include "TPIKDefines.h"
#include <iostream>
#include <vector>

namespace tpik {
/**
 * @brief Hierarchy typedef.
 */
typedef std::vector<std::shared_ptr<tpik::PriorityLevel>> Hierarchy;

/**
 * @brief Action class.
 * @details Implementation of the Action class. Each action is composed by an ID and a vector of std::shared_ptr to tpik::PriorityLevels defining the
 * action hierarchy.
 */

class Action {
public:
    /**
	 * @brief Default constructor.
	 */
    Action();
    /**
	 * @brief Default de constructor.
	 */
    ~Action();
    /**
     * @brief Methods setting and getting the Action ID.
	 */
    auto ID() -> std::string& { return ID_; }
    auto ID() const -> const std::string& { return ID_; }
    /**
	 * @brief Method checking whether the input tpik::PriorityLevel is present in the tpik::Action.
	 * @param[in] priorityLevel std::shared_ptr to the tpik::PriorityLevel.
	 * @return true if priorityLevel is in the action hierarchy, false otherwise.
	 */
    bool FindPriorityLevel(const std::shared_ptr<PriorityLevel> &priorityLevel);
    /**
	 * @brief Method adding the input tpik::PriorityLevel to the action.
	 * @param[in] priorityLevel shared_ptr to the tpik::PriorityLevel to be added.
	 */
    void AddPriorityLevel(const std::shared_ptr<PriorityLevel> priorityLevel);
    /**
	 * Method returning the action hierarchy;
	 * @return Action Hierarchy.
     */
    auto PriorityLevels() const -> const Hierarchy& { return priorityLevels_; }
    /**
	 * @brief Overloading of the cout operator.
	 */
    friend std::ostream& operator<<(std::ostream& os, Action const& action)
    {
        return os << "\033[1;37m"
                  << "Action ID " << action.ID_ << "\n"
                  << std::setprecision(4);
    }

private:
    Hierarchy priorityLevels_; //!< The action Hierarchy.
    std::string ID_; //!< The action ID.
};
}
#endif
