#include "Action.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

void Action::SetID(std::string ID){
	ID_=ID;
}
bool Action::FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel){
	for (auto& priorityLevelHierarhcy:priorityLevels_){
		if(priorityLevelHierarhcy->GetID()==(priorityLevel->GetID())){
			return true;
		}
	}
	return false;
};

void Action::AddPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel){
	priorityLevels_.push_back(priorityLevel);
};
const std::vector<std::shared_ptr<PriorityLevel> > Action::GetPriorityLevels() const{
	return priorityLevels_;
};
	std::string Action::GetID(){
		return ID_;
	};
