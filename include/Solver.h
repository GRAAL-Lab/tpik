#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ActionManager.h"
#include "TPIK.h"
#include "TPIKExceptions.h"

namespace tpik{
class Solver{
public:
	Solver();
	Solver( std::shared_ptr<ActionManager> actionManager,std::shared_ptr<TPIK> tpik );//,
	void SetActionManager(std::shared_ptr<ActionManager> actionManager);
	void SetTPIK(std::shared_ptr<TPIK> tpik);
	void SetAction(std::string action)throw (SolverNotInitializationException);
	const Eigen::VectorXd ComputeVelocities()throw (SolverNotInitializationException);
	friend std::ostream& operator <<(std::ostream& os, Solver const& solver){
		return os<< "\033[1;37m"<<"Solver \n"<<std::setprecision(2)
		<<*solver.actionManager_<<"\n"<<*solver.tpik_;
	}
private :
	std::shared_ptr<ActionManager> actionManager_;
	std::shared_ptr<TPIK> tpik_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
};
}

#endif
