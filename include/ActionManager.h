#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "Action.h"
#include "PriorityLevel.h"
#include <chrono>
#include "TPIKExceptions.h"
#include "TPIKDefines.h"


namespace tpik{
class ActionManager{
public:
	ActionManager( std::vector<std::shared_ptr<PriorityLevel> > hierarchy);
	ActionManager();
	void AddAction(std::shared_ptr<Action> action);
	std::shared_ptr<Action> FindAction(std::vector<std::shared_ptr<Action> > actions, std::string ID);
	void SetAction(std::string newAction) throw (ActionManagerNullActionException);
	void ComputeExternalActivation() const throw (ActionManagerHierarchyException);
    const std::vector<std::shared_ptr<PriorityLevel> >& GetHierarchy() const throw (ActionManagerHierarchyException);
    void SetHierarchy(std::vector<std::shared_ptr<PriorityLevel> > hierarchy);
	friend std::ostream& operator <<(std::ostream& os, ActionManager const& actionManager){
		std::time_t ttp = std::chrono::system_clock::to_time_t(actionManager.time_);

		return os << "\033[1;37m" << "ActionManager \n" << std::setprecision(2)
		<< "Current Action " << "\033[0m" << *actionManager.currentAction_
		<< "\033[1;37m" << "OldAction " << "\033[0m" << *actionManager.oldAction_
		<< "\033[1;37m" << "Time Elapsed " << "\033[0m" << std::put_time(std::localtime(&ttp), "%F %T");
	};

protected:
	std::vector<std::shared_ptr<Action>> actions_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
	std::shared_ptr<Action> currentAction_;
	std::shared_ptr<Action> oldAction_;
	std::chrono::system_clock::time_point time_;

};
}

#endif








