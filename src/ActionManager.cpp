#include "ActionManager.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

ActionManager::ActionManager(std::vector<std::shared_ptr<PriorityLevel> > hierarchy){
	hierarchy_=hierarchy;
	time_=0;

}

void ActionManager::AddAction(Action action){
	actions_.push_back(action);
};

bool  ActionManager::FindAction(std::vector<Action> actions, std::string ID, Action *action){
	for(auto& act:actions){
		if (act.ID==ID){
			action=&act;
			return true ;
		}
	}
	return false;


};

void ActionManager::SetAction(std::string newAction){
	if(!ActionManager::FindAction(actions_,currentAction_.ID,&oldAction_)){
		//warning, the action is not present in the set of actions
		// if it is found automatically the old action became the current action
	}
	else {

	}
	if(!ActionManager::FindAction(actions_,newAction,&currentAction_)){
		//Warning, the action is not present in the set of actions
		// if it is find the current action became equal to the newAction
	}
	//todo put real function
	//time_=TimeMillisecond();
};
void ActionManager::ComputeExternalActivation(){
	for(auto& priorityLevelHierarchy:hierarchy_){
		int numberOfTask=priorityLevelHierarchy->GetNumberOfTask();
		Eigen::MatrixXd I=Eigen::MatrixXd::Identity(numberOfTask,numberOfTask);
		bool isInOldAction=oldAction_.FindPriorityLevel(priorityLevelHierarchy);
		bool isInNewAction=currentAction_.FindPriorityLevel(priorityLevelHierarchy);

		//bool isInOldAction=FindPriorityLevel(priorityLevelHierarchy,oldAction_.priorityLevels);
		//bool isInNewAction=FindPriorityLevel(priorityLevelHierarchy,currentAction_.priorityLevels);

		if(isInOldAction&&isInNewAction){
			// The PL is already active :identity
			priorityLevelHierarchy->SetExternalActivationFunction(I);
		}
		else if(!isInOldAction&&isInNewAction){
			// The PL is to be activated :increasing
			priorityLevelHierarchy->SetExternalActivationFunction(I);
		}
		else if (isInOldAction&&!isInNewAction){
			//The PL must be deactivated: decreasing
			priorityLevelHierarchy->SetExternalActivationFunction(I);
		}
		else {
			//The PL is already deactivated:zeros
			priorityLevelHierarchy->SetExternalActivationFunction(I*0.0);
		}
	}

}

const std::vector<std::shared_ptr<PriorityLevel> >& ActionManager::GetHierarchy() const{
	return hierarchy_;
};
