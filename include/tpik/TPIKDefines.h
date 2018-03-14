#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__

namespace tpik
{

struct BellShapedParameter
{
	double deltaMinBound;
	double deltaMaxBound;
};

struct TaskParameter
{
	double gain;
	bool TaskEnable;
};

}

#endif
