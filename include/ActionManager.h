#ifndef __ACTIONMANAGER_H__
#define __ACTIONMANAGER_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "Action.h"
#include "PriorityLevel.h"



class ActionManager{
public:
	ActionManager( std::vector<std::shared_ptr<PriorityLevel> > hierarchy);
	void AddAction(Action action);
	bool FindAction(std::vector<Action> actions, std::string ID, Action *action);
	void SetAction(std::string newAction);
	void ComputeExternalActivation();
    const std::vector<std::shared_ptr<PriorityLevel> >& GetHierarchy()const;

protected:
	std::vector<Action> actions_;
	std::vector<std::shared_ptr<PriorityLevel> > hierarchy_;
	Action currentAction_;
	Action oldAction_;
	double time_;


};

#endif








