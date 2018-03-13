#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ActionManager.h"
#include "TPIK.h"
#include "TPIKExceptions.h"

namespace tpik
{
/**
 * @brief Solver class.
 * Implementation of the Solver class. Using the tpik::ActionManager and tpik::TPIK, it computes the inverse kinematic control for all the priorityLevels
 * by taking into account the transition between the current action and the previous one.
 */
class Solver
{

public:
	/**
	 * @brief Solver Class constructor.
	 * @param[in] actionManger: std::shared_ptr to the action manager;
	 * @param[in] tpik: std::shared_ptr to tpi
	 * k.
	 */
	Solver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Method which sets the tpik.
	 * @param[in] tpik: std::shared_ptr to tpik.
	 */
	void SetTPIK(std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Method which sets the current action.
	 * @param[in] action: current action ID.
	 */
	void SetAction(std::string action);
	/**
	 * @brief Method which implements the kinematic control by computing and returning the desired velocities.
	 * @return Computed Velocity Vector.
	 */
	const Eigen::VectorXd ComputeVelocities();
	/**
	 * @brief Overloading of the cout operator
	 */
	friend std::ostream& operator <<(std::ostream& os, Solver const& solver)
	{
		return os << "\033[1;37m" << "Solver \n" << std::setprecision(2) << *solver.actionManager_
				<< "\n" << *solver.tpik_;
	}
private:
	std::shared_ptr<ActionManager> actionManager_;
	std::shared_ptr<TPIK> tpik_;
	Hierarchy hierarchy_;
};
}

#endif
