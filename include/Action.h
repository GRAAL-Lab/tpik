#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "PriorityLevel.h"

namespace tpik{
class Action{
public:

	void SetID(std::string ID);
	bool FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	void AddPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	const std::vector<std::shared_ptr<PriorityLevel> > GetPriorityLevels() const;
	std::string GetID();
	friend std::ostream& operator <<(std::ostream& os, Action const& action){
		return os<< "\033[1;37m"<<"Action ID "<<action.ID_<<"\n"<<std::setprecision(2);};
private:
	std::vector<std::shared_ptr<PriorityLevel> > priorityLevels_;
	std::string ID_;
};
}
#endif
