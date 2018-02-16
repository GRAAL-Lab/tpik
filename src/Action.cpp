#include "Action.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>


bool Action::FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel){
	for (auto& priorityLevelHierarhcy:priorityLevels){
		if(priorityLevelHierarhcy->GetID().compare(priorityLevel->GetID())){
			return true;
		}
	}
	return false;
};
