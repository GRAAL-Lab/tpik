#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__

namespace tpik {

enum class TaskType {
	Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween
};

struct BellShapedParameter {
	double sigma1Max;
	double sigma2Max;
	double sigma1Min;
	double sigma3Min;
};

struct TaskParameter {
	double gain;
	bool TaskEnable;
};

}

#endif
