#include "ActionManager.h"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>


tpik::ActionManager::ActionManager(std::vector<std::shared_ptr<PriorityLevel> > hierarchy){
	hierarchy_=hierarchy;
	auto defaultAct= std::make_shared<Action>(Action());
	defaultAct->SetID("DEFAULT_ACTION");
	oldAction_=defaultAct;
	currentAction_=defaultAct;
	actions_.push_back(oldAction_);
}

tpik::ActionManager::ActionManager(){
	auto defaultAct= std::make_shared<Action>(Action());
	defaultAct->SetID("DEFAULT_ACTION");
	oldAction_=defaultAct;
	currentAction_=defaultAct;
	actions_.push_back(oldAction_);
}

void tpik::ActionManager::AddAction(std::shared_ptr<Action> action){
	actions_.push_back(action);
};

std::shared_ptr<tpik::Action>  tpik::ActionManager::FindAction(std::vector<std::shared_ptr<Action> > actions, std::string ID){
	for(auto& act:actions){
		if (act->GetID()==ID){
			return act;
		}
	}
	return nullptr;
};

void tpik::ActionManager::SetAction(std::string newAction) throw (ActionManagerNullActionException){
	oldAction_=currentAction_;
	currentAction_=FindAction(actions_,newAction);
	if(currentAction_==nullptr){
		throw(ActionManagerNullActionException());
	}
	time_=std::chrono::system_clock::now();
};

void tpik::ActionManager::ComputeExternalActivation() const throw (ActionManagerHierarchyException){
	if(hierarchy_.empty()){
		throw (ActionManagerHierarchyException());
	}
	for(auto& priorityLevel:hierarchy_){
		bool isInOldAction=oldAction_->FindPriorityLevel(priorityLevel);
		bool isInNewAction=currentAction_->FindPriorityLevel(priorityLevel);
		double Ae;
		if(isInOldAction&&isInNewAction){
			// The PL is already active :identity
			priorityLevel->SetExternalActivationFunction(1.0);
		}
		else if(!isInOldAction&&isInNewAction){
			//increasing
			std::chrono::duration<double,std::milli> diff = std::chrono::system_clock::now()-time_;
			std::cout<<diff.count()<<" DIFF"<<std::endl;
			Ae=rml::IncreasingBellShapedFunction(0.00,500,0,1,diff.count());
			priorityLevel->SetExternalActivationFunction(Ae);
			std::cout<<Ae<<" AE"<<std::endl;
		}
		else if (isInOldAction&&!isInNewAction){
			std::chrono::duration<double,std::milli> diff = std::chrono::system_clock::now()-time_;
			//The PL must be deactivated: decreasing
			std::cout<<diff.count()<<" DIFF"<<std::endl;
			Ae=rml::DecreasingBellShapedFunction(0.00,500,0,1,diff.count());
			priorityLevel->SetExternalActivationFunction(Ae);
			std::cout<<Ae<<" AE"<<std::endl;

		}
		else {
			//The PL is already deactivated:zeros
			priorityLevel->SetExternalActivationFunction(0.0);
		}
	}

}

const std::vector<std::shared_ptr<tpik::PriorityLevel> >& tpik::ActionManager::GetHierarchy() const throw (ActionManagerHierarchyException){
	if (hierarchy_.empty()){
		throw ActionManagerHierarchyException();
	}
	return hierarchy_;
};

void tpik::ActionManager::SetHierarchy(std::vector<std::shared_ptr<PriorityLevel> > hierarchy){
	hierarchy_=hierarchy;
};
