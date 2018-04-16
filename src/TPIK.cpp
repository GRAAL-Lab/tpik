#include "tpik/TPIK.h"

namespace tpik
{
TPIK::TPIK(int DoF): DoF_(DoF)
{
	I_.setIdentity(DoF_, DoF_);
	y_.setZero(DoF_);
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

}

int TPIK::GetDoF()
{
	return DoF_;
}

}
