#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "PriorityLevel.h"


class Action{
public:
	bool FindPriorityLevel(std::shared_ptr<PriorityLevel> priorityLevel);
	std::vector<std::shared_ptr<PriorityLevel> > priorityLevels;
	std::string ID;
};

#endif
