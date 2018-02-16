#ifndef __SOLVER_H__
#define __SOLVER_H__


#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "ActionManager.h"
#include "TPIK.h"

class Solver{
public:
	Solver( std::shared_ptr<ActionManager> actionManager,std::shared_ptr<TPIK> tpik );//,
	void SetAction(std::string action);
	const Eigen::VectorXd ComputeVelocities();
private :
	std::shared_ptr<ActionManager> actionManager_;
	std::shared_ptr<TPIK> tpik_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
};

#endif
