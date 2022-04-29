#include "tpik/TPIK.h"
namespace tpik {
TPIK::TPIK(int DoF)
    : DoF_(DoF)
    , isSaturationSet_ { false }
{
    I_.setIdentity(DoF_, DoF_);
    y_.setZero(DoF_);
    deltaY_.setZero(DoF_);
    Q_ = I_;
}

TPIK::~TPIK() { }

void TPIK::Reset()
{
    y_.setZero();
    Q_.setIdentity();
    deltaY_.setZero();
    saturationMax_ = originalSaturationMax_;
    saturationMin_ = originalSaturationMin_;
}

void TPIK::SetSaturation(const Eigen::VectorXd& saturationMin, const Eigen::VectorXd& saturationMax)
{
    for (int i = 0; i < DoF_; i++) {
        if (saturationMin(i) > 0) {
            throw std::invalid_argument(std::string("Minimum value for the saturation must be equal or less than zero (DoF: ") + std::to_string(i) + std::string(", value = ") + std::to_string(saturationMin(i)) + std::string(")."));
        }

        if (saturationMax(i) < 0) {
            throw std::invalid_argument(std::string("Maximum value for the saturation must be equal or greater than zero (DoF: ") + std::to_string(i) + std::string(", value = ") + std::to_string(saturationMax(i)) +  std::string(")."));
        }
    }
    saturationMax_ = saturationMax;
    originalSaturationMax_ = saturationMax;
    saturationMin_ = saturationMin;
    originalSaturationMin_ = saturationMin;
    isSaturationSet_ = true;
}

void TPIK::GetSaturation(Eigen::VectorXd& saturationMin, Eigen::VectorXd& saturationMax) const
{
    assert(isSaturationSet_);
    saturationMax = originalSaturationMax_;
    saturationMin = originalSaturationMin_;
}

}
