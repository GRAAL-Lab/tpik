#ifndef __iCAT_H__
#define __iCAT_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "TPIK.h"
#include "TPIKExceptions.h"

namespace tpik {
class iCAT: public TPIK {
public:
	/**
	 * @brief Class constructor
	 * @param[in] DoF: Degrees of Freedom.
	 *  */
	iCAT(int DoF);
	/**
	 * @brief Class default constructor.
	 *  */
	iCAT();
	/**
	 * @brief Class default de-constructor.
	 *  */
	virtual ~iCAT();
	/**
	 * @brief Implementation of the pure virtual method  that computes the kinematic control for a single priority level.
	 * To be implemented in the derived classes.
	 * @param[in]
	 **/

	/**
	 *
	 * @param J
	 * @param Alpha
	 * @param x_dot
	 * @param svd
	 */
	virtual void ComputeYStep(Eigen::MatrixXd J, Eigen::MatrixXd Alpha,
			Eigen::VectorXd x_dot, rml::SVDParameters svd)
					throw (TPIKMissingDoFInitializationException);
};
}
#endif
