#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "PriorityLevel.h"


class Action{
public:

	void SetID(std::string ID);
	bool FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	void AddPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	const std::vector<std::shared_ptr<PriorityLevel> > GetPriorityLevels() const;
	std::string GetID();
private:
	std::vector<std::shared_ptr<PriorityLevel> > priorityLevels_;
	std::string ID_;
};

#endif
