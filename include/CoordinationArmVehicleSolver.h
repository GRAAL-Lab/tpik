#ifndef __ARMVEHICLESOLVER_H__
#define __ARMVEHICLESOLVER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <tpik/TPIKlib.h>
#include <rml/RML.h>

namespace tpik {
class CoordinationArmVehicleSolver {
public:
	CoordinationArmVehicleSolver(std::shared_ptr<tpik::ActionManager> actionManager, std::shared_ptr<tpik::TPIK> tpik,
			std::shared_ptr<Task> vehicleTask, rml::SVDParameters vehicleTaskSVDParameter);
	void SetAction(std::string action);
	void SetTPIK(std::shared_ptr<tpik::TPIK> tpik);
	void SetActionManager(std::shared_ptr<tpik::ActionManager> actionManager);
	void SetVehicleTask(std::shared_ptr<Task> vehicleTask);
	const Eigen::VectorXd ComputeDecoupledVelocities();
private:
	std::shared_ptr<tpik::ActionManager> actionManager_;
	std::shared_ptr<tpik::TPIK> tpik_;
	std::vector<std::shared_ptr<tpik::PriorityLevel> > hierarchy_;
	std::vector<std::shared_ptr<tpik::PriorityLevel> > hierarchyArm_;
	std::shared_ptr<Task> vehicleTask_;
	std::shared_ptr<tpik::PriorityLevel> vehiclePL_;
};
}
#endif
