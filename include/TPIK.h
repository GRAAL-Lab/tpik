#ifndef __TPIK_H__
#define __TPIK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "PriorityLevel.h"

namespace tpik {
/**
 * @brief TPIK class
 * Implementation of the TPIK (Task Priority Inverse Kinematic) Abstract class containing the pure virtual method ComputeYStep
 * to compute the inverse kinematic control for a single priority level
 *  */
class TPIK {
public:
	/**
	 * @brief TPIK constructor
	 * @param[in] DoF: degrees of freedom;
	 *  */
	TPIK(int DoF);
	/**
	 * @brief TPIK Default constructor
	 *  */
	TPIK();
	/**
	 * @brief TPIK virtual Default de-constructor
	 *  */
	virtual ~TPIK();
	/**
	 * @brief Pure virtual function that computes the kinematic control for a single priority level.
	 * To be implemented in the derived classes.
	 * @param[in] J: Jacobian Matrix;
	 * @param[in] Alpha: Activation Function;
	 * @param[in] x_dot: Reference;
	 * @param[in] svd: svd Parameters
	 *  */
	virtual void ComputeYStep(Eigen::MatrixXd J, Eigen::MatrixXd Alpha, Eigen::VectorXd x_dot,
			rml::SVDParameters svd)=0;
	/**
	 * @brief Function which return the computed velocity
	 * @return Inverse Kinematic Velocity
	 *  */
	const Eigen::VectorXd& GetY() const;
	/**
	 * @brief Function which reset the class variables in order to compute a new kinematic control
	 *  */
	void Reset();
	/**
	 * @brief Function which sets the system Degrees of Freedom
	 * @param[in] DoF: Degrees of Freedom.
	 *  */
	void SetDoF(int DoF);
	/**
	 * @brief Function which returns the system Degrees of Freedom
	 * @return Degrees of Freedom
	 *  */
	int GetDoF();
	/**
	 * @brief Overload of the cout function
	 *  */
	friend std::ostream& operator <<(std::ostream& os, TPIK const& tpik) {
		return os << "\033[1;37m" << "TPIK" << "\n" << std::setprecision(2)
				<< "\033[1;37m" << "Y \n" << "\033[0m" << tpik.y_ << "\n" << "\033[1;37m"
				<< "Q \n" << "\033[0m" << tpik.Q_ << "\n";
	}


protected:
	Eigen::VectorXd y_;
	Eigen::MatrixXd Q_;
	Eigen::MatrixXd I_;
	int DoF_;
};
}

#endif
