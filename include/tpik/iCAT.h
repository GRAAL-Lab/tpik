#ifndef __iCAT_H__
#define __iCAT_H__

#include "TPIK.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace tpik {
/**
 * @brief iCAT Class derived from the tpik::TPIK abstract class.
 * @details Implementation of the iCAT (Inequality Constraints Activation and Task) algorithm. It implements the pure virtual method ComputeYStep of
 * the tpik::TPIK class to compute the inverse kinematic of a single priority level.
 */
class iCAT : public TPIK {
public:
    /**
	 * @brief Class constructor.
	 * @param[in] DoF: Degrees of Freedom.
	 */
    iCAT(int DoF);
    /**
	 * @brief Class default de-constructor.
	 */
    ~iCAT() override;
    /**
	 * @brief Implementation of the pure virtual method that computes the inverse kinematic control for a single priority level.
	 * @param[in] J: Jacobian Matrix;
	 * @param[in] A: Activation Function (Ai*Ae);
	 * @param[in] x_dot: Reference;
	 * @param[in] regularizationData rml::RegularizationData struct.
	 */
    void ComputeVelocities(const Eigen::MatrixXd& J, const Eigen::MatrixXd& A, const Eigen::VectorXd& x_dot, rml::RegularizationData& regularizationData) override;

protected:
    /**
   * @brief Method saturating the y taking into account the saturation performed for the higher priority levels.
   */
    void Saturate();
};
}
#endif
