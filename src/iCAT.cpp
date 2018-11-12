#include "tpik/iCAT.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RML.h>
#include <vector>

namespace tpik {
iCAT::iCAT(int DoF)
    : TPIK(DoF)
{
}

iCAT::~iCAT()
{
}

void iCAT::ComputeYSingleLevel(Eigen::MatrixXd J, Eigen::MatrixXd A, Eigen::VectorXd xdot,
    rml::RegularizationData regularizationData)
{

    if (!A.isZero()) {
        Eigen::MatrixXd barG = J * Q_;
        Eigen::MatrixXd barGtraspAA = barG.transpose() * A * A;
        Eigen::MatrixXd T = (I_ - Q_).transpose() * (I_ - Q_);
        Eigen::MatrixXd H = barG.transpose() * (Eigen::MatrixXd::Identity(J.rows(), J.rows()) - A) * A * barG;
        Eigen::MatrixXd W = barG
            * rml::RegularizedPseudoInverse((Eigen::MatrixXd)(barGtraspAA * barG + T + H), regularizationData)
            * barGtraspAA;
        Eigen::MatrixXd barGpinv = rml::RegularizedPseudoInverse((Eigen::MatrixXd)(barGtraspAA * barG + H),
            regularizationData);
        deltaY_ = Q_ * barGpinv * barGtraspAA * W * (xdot - J * y_);

       Saturate();

        y_ = y_ + deltaY_;
        Q_ = Q_ * (I_ - barGpinv * barGtraspAA * barG);
    } else {
        deltaY_.setZero();
    }
}

void iCAT::Saturate()
{

    double min_factor = 1.0;
    for (int i = 0; i < DoF_; i++) {
        double factor = 1.0;
        if (deltaY_(i) > saturationMax_(i)) {
            if (deltaY_(i) != 0.0) {
                factor = std::fabs(saturationMax_(i) / deltaY_(i));
            }
        } else if (deltaY_(i) < saturationMin_(i)) {
            if (deltaY_(i) != 0.0) {
                factor = std::fabs(saturationMin_(i) / deltaY_(i));
            }
        }
        if (factor < min_factor) {
            min_factor = factor;
        }
    }

    for (int i = 0; i < DoF_; i++) {
        deltaY_(i) = deltaY_(i) * min_factor;
        saturationMax_(i) = saturationMax_(i) - deltaY_(i);
        saturationMin_(i) = saturationMin_(i) - deltaY_(i);
    }
}
}
///protected
