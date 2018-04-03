#ifndef __TPIK_H__
#define __TPIK_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "PriorityLevel.h"

namespace tpik {
/**
 * @brief TPIK class.
 * Implementation of the TPIK (Task Priority Inverse Kinematic) Abstract class containing the pure virtual method ComputeYSingleLevel
 * to compute the inverse kinematic control for a single priority level.
 */
class TPIK {
public:
	/**
	 * @brief TPIK constructor.
	 * @param[in] DoF: degrees of freedom;
	 */
	TPIK(int DoF);

	/**
	 * @brief TPIK virtual Default de-constructor.
	 */
	virtual ~TPIK();
	/**
	 * @brief Pure virtual method that computes the kinematic control for a single priority level.
	 * To be implemented in the derived classes.
	 * @param[in] J: Jacobian Matrix;
	 * @param[in] A: Activation Function (Ai*Ae);
	 * @param[in] x_dot: Reference;
	 * @param[in] regularizationData: rml::RegularizationData struct.
	 */
	virtual void ComputeYSingleLevel(Eigen::MatrixXd J, Eigen::MatrixXd A, Eigen::VectorXd x_dot,
			rml::RegularizationData regularizationData)=0;
	/**
	 * @brief Method which returns the computed velocity.
	 * @return Inverse Kinematic Velocity.
	 */
	const Eigen::VectorXd& GetY() const;
	/**
	 * @brief Method which resets the class variables in order to compute a new kinematic control.
	 */
	void Reset();
	/**
	 * @brief Method which returns the system Degrees of Freedom
	 * @return Degrees of freedom.
	 */
	int GetDoF();
	/**
	 * @brief Overload of the cout function
	 */
	friend std::ostream& operator <<(std::ostream& os, TPIK const& tpik) {
		return os << "\033[1;37m" << "TPIK" << "\n" << std::setprecision(2) << "\033[1;37m"
				<< "Y \n" << "\033[0m" << tpik.y_ << "\n" << "\033[1;37m"
				<< "Q \n" << "\033[0m" << tpik.Q_ << "\n";
	}

protected:
	Eigen::VectorXd y_; //!< The velocity.
	Eigen::MatrixXd Q_;//!< The Q matrix stating the space in which the following velocities must be generated in order not to affect the velocities generated for the higher priority levels.
	Eigen::MatrixXd I_;//!< The identity Matrix (DoF x DoF).
	int DoF_; //!< The degrees of freedom.
};
}

#endif
