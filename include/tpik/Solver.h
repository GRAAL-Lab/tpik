#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ActionManager.h"
#include "TPIK.h"

namespace tpik
{
/**
 * @brief Solver class.
 * @details Implementation of the Solver class. Using the tpik::ActionManager and tpik::TPIK, it computes the inverse kinematic control for all the priorityLevels
 * by taking into account the transition between the current action and the previous one.
 */
class Solver
{

public:
	/**
	 * @brief Solver Class constructor.
	 * @param[in] actionManager std::shared_ptr to the tpik::ActionManager containing the unified hierarchy and the action list. Before calling the constructor the
	 * unified hierarchy must already have been specified.
	 * @param[in] tpik std::shared_ptr to tpik::TPIK.
	 *
	 */
	Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Method which sets the tpik.
	 * @param[in] tpik std::shared_ptr to TPIK::TPIK.
	 * @note Such method coul dbe used to change algorithm for computing the kinematic control for a single priority level.
	 */
	void SetTPIK(std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Method which sets the current action.
	 * @param[in] action current action ID.
	 */
	void SetAction(std::string action);
	/**
	 * @brief Method which implements the kinematic control by computing and returning the desired velocities.
	 * @return Computed Velocity Vector.
	 */
    const Eigen::VectorXd& ComputeVelocities();
    /**
     * @brief Method returning the delta y computed at each level.
     * @return std vector of eigen vector containing the y increments computed for each priority level.
     */
    std::vector<Eigen::VectorXd> GetDeltaYs();
	/**
	 * @brief Overloading of the cout operator
	 */
	friend std::ostream& operator <<(std::ostream& os, Solver const& solver)
	{
		return os << "\033[1;37m" << "Solver \n" << std::setprecision(2) << *solver.actionManager_ << "\n"
				<< *solver.tpik_;
	}
private:
	std::shared_ptr<ActionManager> actionManager_; //!< The std::shared_ptr to the tpik::ActionManager.
	std::shared_ptr<TPIK> tpik_; //!< The std::shared_ptr to the tpik::TPIK.
	Hierarchy hierarchy_; //!< The unified hierarhcy.
    std::vector<Eigen::VectorXd> delta_y; //!<y computed at each priority level.
};
}

#endif
