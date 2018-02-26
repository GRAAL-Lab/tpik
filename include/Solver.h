#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ActionManager.h"
#include "TPIK.h"
#include "TPIKExceptions.h"

namespace tpik {
class Solver {

public:
	/**
	 * @brief Solver Class default constructor.
	 *  */
	Solver();
	/**
	 * @brief Solver Class constructor.
	 * @param[in] actionManger: shared_ptr to the action manager;
	 * @param[in] tpik: shared_ptr to tpik.
	 *  */
	Solver(std::shared_ptr<ActionManager> actionManager,
			std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Function which sets the action Manager
	 * @param[in] actionManger: shared_ptr to the action manager.
	 *  */
	void SetActionManager(std::shared_ptr<ActionManager> actionManager);
	/**
	 * @brief Function which sets the tpik
	 * @param[in] tpik: shared_ptr to tpik.
	 *  */
	void SetTPIK(std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Function which sets the current action
	 * @param[in] action: current action ID.
	 *  */
	void SetAction(std::string action) throw (SolverNotInitializationException);
	/**
	 * @brief Function which implements the kinematic control by computing and returning the desired velocities.
	 *  */
	const Eigen::VectorXd ComputeVelocities()
			throw (SolverNotInitializationException);
	/**
	 * @brief Overloading of the cout operator
	 *  */
	friend std::ostream& operator <<(std::ostream& os, Solver const& solver) {
		return os << "\033[1;37m" << "Solver \n" << std::setprecision(2)
				<< *solver.actionManager_ << "\n" << *solver.tpik_;
	}
private:
	std::shared_ptr<ActionManager> actionManager_;
	std::shared_ptr<TPIK> tpik_;
	Hierarchy hierarchy_;
};
}

#endif
