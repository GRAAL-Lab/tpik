#ifndef __ARMVEHICLESOLVER_H__
#define __ARMVEHICLESOLVER_H__

#include "ActionManager.h"
#include "TPIK.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace tpik {
/**
 * @brief CoordinationArmVehicleSolver class.
 * @details Implementation of the CoordinationArmVehicleSolver class. Such class computes the kinematic control for a vehicle
 * and arms by optimizing the arms velocities wrt to the vehicle current velocities.\n
 * In order to achieve such result, an implementation of the Task class which provides the vehicle teleoperation
 * with the actual vehicle velocities is needed.\n
 * First the whole system velocities (both for the vehicle and the arms) are computed. Then, by taking into account the current vehicle
 * velocities, the arms velocities are computed hence obtaining the optimized arms velocities wrt to the current vehicle velocities.\n
 * The desired vehicle velocities returned will be the one of the first optimization while the arms velocities will be the one of the second otpimization.
 * In this way the difference in between the dynamic and kinematic of the arm and the vehicle is taken into account and the arms target velocities
 * will be independent on the possible vehicle errors in tracking the desired velocities.\n
 * @note the vehicle teleoperation task must NOT be included in the unified heirarchy.
 */
class CoordinationArmVehicleSolver {
public:
    /**
	 * @brief Overload of the consturctur
	 * @param[in] actionManager std::shared_ptr to tpik::ActionManager which manages the unified hierarchy and the mission action.
	 * @param[in] tpik std::shared_ptr to tpik::TPIK which provides the method to solve a single tpik::PriorityLevel.
	 * @param[in] vehiclePriorityLevel std::shared_ptr to tpik::PriorityLevel containing task providing the teleoparation for the vehicle velocities
	 * (with regularization data already set ).
	 */
    CoordinationArmVehicleSolver(const std::shared_ptr<ActionManager> actionManager, const std::shared_ptr<TPIK> tpik, const std::shared_ptr<PriorityLevel> vehiclePriorityLevel);
    /**
	 * @brief Method setting the current tpik::Action.
	 * @param[in] action current action ID.
     * @param[in] transaaction true if transition false otherwise
	 */

    bool SetAction(const std::string action, bool transition);
    /**
	 * @brief Method that computes the optimized velocities for the whole system through two different optimization. One for the vehicle and
	 * on for the arms.
	 * @return Kinematic control velocities.
	 */
    const Eigen::VectorXd ComputeDecoupledVelocities();

private:
    std::shared_ptr<ActionManager> actionManager_; //!< The std::shared_ptr to the tpik::ActionManager.
    std::shared_ptr<TPIK> tpik_; //!< The std::shared_ptr to the tpik::TPIK.
    std::vector<std::shared_ptr<PriorityLevel>> hierarchy_; //!< The unified hierarhcy.
    std::vector<std::shared_ptr<PriorityLevel>> hierarchyArm_; //!< The unified hierarchy used to compute the arm velocites, hence with the vehicle teleopertion.
    std::shared_ptr<PriorityLevel> vehiclePriorityLevel_; //!< The std::shared_ptr to vehicle teleoperatio tpik::PriorityLevel.
};
}
#endif
