#ifndef __iCAT_H__
#define __iCAT_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "TPIK.h"
#include "TPIKExceptions.h"

namespace tpik
{
/**
 * @brief iCAT Class derived from the tpik::TPIK abstract class.
 * Implementation of the iCAT (Inequality Constraints Activation and Task) algorithm. It implements the pure virtual method ComputeYStep of
 * the tpik::TPIK class to compute the inverse kinematic of a single task level.
 */
class iCAT: public TPIK
{
public:
	/**
	 * @brief Class constructor.
	 * @param[in] DoF: Degrees of Freedom.
	 */
	iCAT(int DoF);

	/**
	 * @brief Class default de-constructor.
	 */
	virtual ~iCAT();
	/**
	 * @brief Implementation of the pure virtual method that computes the inverse kinematic control for a single priority level.
	 * @param[in] J: Jacobian Matrix;
	 * @param[in] A: Activation Function;
	 * @param[in] x_dot: Reference;
	 * @param[in] svd: rml::SVDParameters
	 */
	virtual void ComputeYSingleLevel(Eigen::MatrixXd J, Eigen::MatrixXd A, Eigen::VectorXd x_dot,
			rml::SVDData svd);
};
}
#endif
