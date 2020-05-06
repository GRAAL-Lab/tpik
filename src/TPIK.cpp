#include "tpik/TPIK.h"

namespace tpik {
TPIK::TPIK(int DoF)
    : DoF_(DoF)
{
    I_.setIdentity(DoF_, DoF_);
    y_.setZero(DoF_);
    deltaY_.setZero(DoF_);
    Q_ = I_;
}

TPIK::~TPIK() {}

void TPIK::Reset()
{
    y_.setZero();
    Q_.setIdentity();
    deltaY_.setZero();
    saturationMax_ = originalSaturationMax_;
    saturationMin_ = originalSaturationMin_;
}
void TPIK::SetSaturation(const Eigen::VectorXd saturationMin, const Eigen::VectorXd saturationMax)
{
    saturationMax_ = std::move(saturationMax);
    originalSaturationMax_ = std::move(saturationMax);
    saturationMin_ = std::move(saturationMin);
    originalSaturationMin_ = std::move(saturationMin);
}
void TPIK::GetSaturation(Eigen::VectorXd& saturationMax, Eigen::VectorXd& saturationMin) const
{
    saturationMax = originalSaturationMax_;
    saturationMin = originalSaturationMin_;
}
}
