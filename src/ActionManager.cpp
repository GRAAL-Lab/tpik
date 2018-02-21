#include "ActionManager.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

ActionManager::ActionManager(std::vector<std::shared_ptr<PriorityLevel> > hierarchy){
	hierarchy_=hierarchy;
	time_=0;
	auto defaultAct= std::make_shared<Action>(Action());
	defaultAct->SetID("DEFAULT_ACTION");
	oldAction_=defaultAct;
	currentAction_=defaultAct;
	actions_.push_back(oldAction_);

}
ActionManager::ActionManager(){
	time_=0;
	auto defaultAct= std::make_shared<Action>(Action());
	defaultAct->SetID("DEFAULT_ACTION");
	oldAction_=defaultAct;
	currentAction_=defaultAct;
	actions_.push_back(oldAction_);
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
	return nullptr;
};

void ActionManager::SetAction(std::string newAction) throw (ActionManagerNullActionException){
	oldAction_=currentAction_;
	currentAction_=FindAction(actions_,newAction);
	if(currentAction_==nullptr){
		throw(ActionManagerNullActionException());
	}
};

void ActionManager::ComputeExternalActivation() const throw (ActionManagerHierarchyException){
	if(hierarchy_.empty()){
		throw (ActionManagerHierarchyException());
	}
	for(auto& priorityLevel:hierarchy_){
		bool isInOldAction=oldAction_->FindPriorityLevel(priorityLevel);
		bool isInNewAction=currentAction_->FindPriorityLevel(priorityLevel);
		//TODO delete used only to debug
		double Ae=0.2;
		/*
		std::cout<<"oldActID "<<oldAction_->GetID()<<std::endl;
		std::cout<<"newActID "<<currentAction_->GetID()<<std::endl;
		std::cout<<"isInOldAction "<<isInOldAction<<std::endl;
		std::cout<<"isInNewAction  "<<isInNewAction<<std::endl;
		*/
		//TODO find delete
		if(isInOldAction&&isInNewAction){
			// The PL is already active :identity
			priorityLevel->SetExternalActivationFunction(1.0);
		}
		else if(!isInOldAction&&isInNewAction){
			// The PL is to be activated :increasing
			priorityLevel->SetExternalActivationFunction(Ae);
		}
		else if (isInOldAction&&!isInNewAction){
			//The PL must be deactivated: decreasing
			priorityLevel->SetExternalActivationFunction(Ae);
		}
		else {
			//The PL is already deactivated:zeros
			priorityLevel->SetExternalActivationFunction(0.0);
		}
	}

}

const std::vector<std::shared_ptr<PriorityLevel> >& ActionManager::GetHierarchy() const throw (ActionManagerHierarchyException){
	if (hierarchy_.empty()){
		throw ActionManagerHierarchyException();
	}
	return hierarchy_;
};

void ActionManager::SetHierarchy(std::vector<std::shared_ptr<PriorityLevel> > hierarchy){
	hierarchy_=hierarchy;
};
