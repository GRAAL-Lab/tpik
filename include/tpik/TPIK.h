#ifndef __TPIK_H__
#define __TPIK_H__

#include "PriorityLevel.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

namespace tpik {
/*
* @brief TPIK class.
* @details Implementation of the TPIK (Task Priority Inverse Kinematic) Abstract class to handle the inverse kinematic control for a single priority level.
*/
class TPIK {
public:
    /*
    * @brief TPIK constructor.
    * @param[in] DoF: degrees of freedom;
    */
    TPIK(int DoF);
    /*
    * @brief TPIK virtual Default de-constructor.
    */
    virtual ~TPIK();
    /*
    * @brief Pure virtual method that computes the kinematic control for a single priority level.
    * To be implemented in the derived classes.
    * @param[in] J: Jacobian Matrix;
    * @param[in] A: Activation Function (Ai*Ae);
    * @param[in] x_dot: Reference;
    * @param[in] regularizationData: rml::RegularizationData struct.
    */
    virtual void ComputeVelocities(const Eigen::MatrixXd& J, const Eigen::MatrixXd& A, const Eigen::VectorXd& x_dot, rml::RegularizationData& regularizationData) = 0;
    /*
    * @brief Method which returns the computed velocity.
    * @return Inverse Kinematic Velocity.
    */
    auto Velocities() const -> const Eigen::VectorXd& { return y_; }
    /*
    * @brief Method which resets the class variables in order to compute a new kinematic control.
    */
    void Reset();
    /*
    * @brief Method which returns the system Degrees of Freedom
    * @return Degrees of freedom.
    */
    auto Dof() const -> int { return DoF_; }
    /*
    * @brief Method setting the saturation values for each dof
    * @param[in] saturationMax maximum value
    * @param[in] saturationMin minimum value
    */
    void SetSaturation(const Eigen::VectorXd saturationMin, const Eigen::VectorXd saturationMax);
    /*
    * @brief Method getting the saturation values for each dof
    * @param[out] saturationMax maximum value
    * @param[out] saturationMin minimum value
    */
    void GetSaturation(Eigen::VectorXd& saturationMin, Eigen::VectorXd& saturationMax) const;
    /*
    * @brief Method which returns the increment of velocities
    */
    auto DeltaY() const -> const Eigen::VectorXd& { return deltaY_; }
    /*
    * @brief Overload of the cout function
    */
    friend std::ostream& operator<<(std::ostream& os, TPIK const& tpik)
    {
        return os << "\033[1;37m"
                  << "TPIK"
                  << "\n"
                  << std::setprecision(4) << "\033[1;37m"
                  << "Y \n"
                  << "\033[0m" << tpik.y_ << "\n"
                  << "\033[1;37m"
                  << "Q \n"
                  << "\033[0m" << tpik.Q_ << "\n";
    }

protected:
    Eigen::VectorXd y_; // The velocity.
    Eigen::MatrixXd Q_; // The Q matrix stating the space in which the following velocities must be generated in order not to affect the velocities generated for the higher priority levels.
    Eigen::MatrixXd I_; // The identity Matrix (DoF x DoF).
    int DoF_; // The degrees of freedom.
    Eigen::VectorXd deltaY_;
    Eigen::VectorXd originalSaturationMax_; // vector containing the initial max value for saturation.
    Eigen::VectorXd originalSaturationMin_; // vector containing the initial min value for saturation.
    Eigen::VectorXd saturationMax_; // vector containing the max value for saturation.
    Eigen::VectorXd saturationMin_; // vector containing the min value for saturation.
};
}

#endif
