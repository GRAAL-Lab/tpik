#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__

#include <eigen3/Eigen/Dense>

namespace tpik
{
/**
 * @brief Parameter used to define a bell shaped function. Used to create either an increasing or a decreasing function with a transitory linear
 * behavior in between xmin and xmax
 */
struct BellShapedParameter
{
	Eigen::VectorXd xmin; //!< Vector containing the xmin for all the bell shaped function described by the struct.
	Eigen::VectorXd xmax; //!< Vector containing the xmax for all the bell shaped function described by the struct.
};

/**
 * @brief Task Parameter, used both in the equality and inequality task
 */
struct TaskParameter
{
	double gain; //!< The reference gain.
	bool taskEnable; //!< Boolean stating whether the task is active
	double saturation; //!< The reference saturation value.
};


}

#endif
