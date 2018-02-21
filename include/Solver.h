#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ActionManager.h"
#include "TPIK.h"
#include "tpikExceptions.h"

class Solver{
public:
	Solver();
	Solver( std::shared_ptr<ActionManager> actionManager,std::shared_ptr<TPIK> tpik );//,
	void SetActionManager(std::shared_ptr<ActionManager> actionManager);
	void SetTPIK(std::shared_ptr<TPIK> tpik);
	void SetAction(std::string action)throw (SolverNotInitializationException);
	const Eigen::VectorXd ComputeVelocities()throw (SolverNotInitializationException);
private :
	std::shared_ptr<ActionManager> actionManager_;
	std::shared_ptr<TPIK> tpik_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
};

#endif
