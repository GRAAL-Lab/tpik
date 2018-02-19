#include "ActionManager.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

ActionManager::ActionManager(std::vector<std::shared_ptr<PriorityLevel> > hierarchy){
	hierarchy_=hierarchy;
	time_=0;
	auto oldAct= std::make_shared<Action>(Action());
	oldAct->SetID("DEFAULT OLD ");
	oldAction_=oldAct;
	actions_.push_back(oldAction_);
	currentAction_=oldAct;

}

void ActionManager::AddAction(std::shared_ptr<Action> action){
	actions_.push_back(action);
};

std::shared_ptr<Action>  ActionManager::FindAction(std::vector<std::shared_ptr<Action> > actions, std::string ID){
	for(auto& act:actions){
		if (act->GetID()==ID){

			return act;
		}
	}
	return false;


};

void ActionManager::SetAction(std::string newAction){
	oldAction_=currentAction_;
	currentAction_=FindAction(actions_,newAction);
	/**
	if(!ActionManager::FindAction(actions_,currentAction_->GetID(),oldAction_)){
		//warning, the action is not present in the set of actions
		// if it is found automatically the old action became the current action
	}
	else {

	}
	if(!ActionManager::FindAction(actions_,newAction,currentAction_)){
		//Warning, the action is not present in the set of actions
		// if it is find the current action became equal to the newAction
	}
	//todo put real function
	//time_=TimeMillisecond();
	 * +//
	 */
};

void ActionManager::ComputeExternalActivation(){
	for(auto& priorityLevelHierarchy:hierarchy_){
		bool isInOldAction=oldAction_->FindPriorityLevel(priorityLevelHierarchy);
		bool isInNewAction=currentAction_->FindPriorityLevel(priorityLevelHierarchy);
		double Ae=0.2;
		std::cout<<"oldActID "<<oldAction_->GetID()<<std::endl;
		std::cout<<"newActID "<<currentAction_->GetID()<<std::endl;
		std::cout<<"isInOldAction "<<isInOldAction<<std::endl;
		std::cout<<"isInNewAction  "<<isInNewAction<<std::endl;
		//bool isInOldAction=FindPriorityLevel(priorityLevelHierarchy,oldAction_.priorityLevels);
		//bool isInNewAction=FindPriorityLevel(priorityLevelHierarchy,currentAction_.priorityLevels);

		if(isInOldAction&&isInNewAction){
			// The PL is already active :identity
			priorityLevelHierarchy->SetExternalActivationFunction(1.0);
		}
		else if(!isInOldAction&&isInNewAction){
			// The PL is to be activated :increasing
			priorityLevelHierarchy->SetExternalActivationFunction(Ae);
		}
		else if (isInOldAction&&!isInNewAction){
			//The PL must be deactivated: decreasing
			priorityLevelHierarchy->SetExternalActivationFunction(Ae);
		}
		else {
			//The PL is already deactivated:zeros
			priorityLevelHierarchy->SetExternalActivationFunction(0.0);
		}
	}

}

const std::vector<std::shared_ptr<PriorityLevel> >& ActionManager::GetHierarchy() const{
	return hierarchy_;
};
