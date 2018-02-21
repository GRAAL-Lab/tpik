#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "Action.h"
#include "PriorityLevel.h"
#include "tpikExceptions.h"


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
protected:
	std::vector<std::shared_ptr<Action>> actions_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
	std::shared_ptr<Action> currentAction_;
	std::shared_ptr<Action> oldAction_;
	double time_;


};

#endif








