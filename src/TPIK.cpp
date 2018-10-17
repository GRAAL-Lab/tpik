#include "tpik/TPIK.h"

namespace tpik
{
TPIK::TPIK(int DoF): DoF_(DoF)
{
	I_.setIdentity(DoF_, DoF_);
	y_.setZero(DoF_);
    deltaY_.setZero(DoF_);
	Q_ = I_;
}

TPIK::~TPIK()
{
}

const Eigen::VectorXd& TPIK::GetY() const
{
	return y_;
}

void TPIK::Reset()
{
	y_.setZero();
	Q_.setIdentity();
  deltaY_.setZero();
  saturationMax_=originalSaturationMax_;
  saturationMin_=originalSaturationMin_;

}
void TPIK::SetSaturation(Eigen::VectorXd saturationMax, Eigen::VectorXd saturationMin)
{
  saturationMax_=saturationMax;
  originalSaturationMax_=saturationMax;
  saturationMin_= saturationMin;
  originalSaturationMin_=saturationMin;

}

int TPIK::GetDoF()
{
	return DoF_;
}
Eigen::VectorXd TPIK::GetDeltaY(){
    return deltaY_;
}

}
