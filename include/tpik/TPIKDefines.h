#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__
#include <eigen3/Eigen/Dense>

namespace tpik
{

struct BellShapedParameter
{
	Eigen::VectorXd xmin;
	Eigen::VectorXd xmax;
};

struct TaskParameter
{
	double gain;
	bool taskEnable;
	double saturation;
};

}

#endif
