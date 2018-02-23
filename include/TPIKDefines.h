#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__

namespace tpik{

enum class TaskType{Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween};

struct BellShapedFunction{
	double sigma1;
	double sigma2;
};

struct TaskParameter{
	BellShapedFunction min,max;
	double gain;
	bool TaskEnable;
};
//typedef std::vector<std::shared_ptr<tpik::PriorityLevel> > Hierarchy;
}

#endif
