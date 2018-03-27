#ifndef __ARMVEHICLESOLVER_H__
#define __ARMVEHICLESOLVER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ActionManager.h"
#include "TPIK.h"

namespace tpik
{
/**
 * @brief CoordinationArmVehicleSolver class.
 * Implementation of the CoordinationArmVehicleSolver class. Such class computes the kinematic control for a vehicle
 * and arms by optimizing the arms velocities wrt to the vehicle current velocities.
 * In order to achieve such results, an implementation of the Task class which provides the vehicle teleoperation
 * with the actual vehicle velocities is needed.
 * First the whole system velocities (both for the vehicle and the arms) are computed. Then, by taking into account the current vehicle
 * velocities, the arms velocities are computed hence obtaining the optimized arms velocities wrt to the current vehicle velocities.
 * The desired vehicle velocities returned will be the one of the first optimization while the arms one will be the one of the second otpimization.
 * In this way the difference in between the dynamic and kinematic of the arm and the vehicle is taken into account and the arms target velocities
 * will be independent on the possible vehicle errors in tracking the desired velocities.
 * @note the vehicle teleoperation task must not be included in the unified heirarchy.
 */
class CoordinationArmVehicleSolver
{
public:
	/**
	 * @brief Overload of the consturctur
	 * @param[in] actionManager: std::shared_ptr to tpik::ActionManager which manages the unified hierarchy and the mission action.
	 * @param[in] tpik: tpik::std::shared_ptr TPIK which provides the method to solve a single priority level.
	 * @param[in] vehicleTask: std::shared_ptr to implementation of the abstract class tpik::Task which provides the teleoparation for the vehicle velocities.
	 * @param[in] vehicleTaskSVDParameter: svd parameters for the input vehicle task.
	 */
	CoordinationArmVehicleSolver(std::shared_ptr<ActionManager> actionManager, std::shared_ptr<TPIK> tpik,
			std::shared_ptr<PriorityLevel> vehiclePriorityLevel, rml::SVDData vehicleTaskSVDParameter);
	/**
	 * @brief Method setting the current action.
	 * @param[in] action: current action ID.
	 */
	void SetAction(std::string action);
	/**
	 * @brief Method setting the tpik .
	 * @param[in] tpik: std::shared_ptr to tpik::TPIK object.
	 */
	void SetTPIK(std::shared_ptr<TPIK> tpik);
	/**
	 * @brief Method that computes the optimized velocities for the whole system through two different optimization. One for the vehicle and
	 * on for the arms.
	 * @return Kinematic Control Velocities.
	 */
	Eigen::VectorXd ComputeDecoupledVelocities();
private:
	std::shared_ptr<ActionManager> actionManager_;
	std::shared_ptr<TPIK> tpik_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchyArm_;
	std::shared_ptr<PriorityLevel> vehiclePriorityLevel_;

};
}
#endif
