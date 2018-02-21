#ifndef __iCAT_H__
#define __iCAT_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "PriorityLevel.h" //to be deleted used only for svd param
#include "tpikExceptions.h"
#include "TPIK.h"

class iCAT : public TPIK {
public:
	iCAT(int DoF);
	iCAT();
	void SetDoF(int DoF);
	virtual ~iCAT();
	virtual void ComputeYStep(Eigen::MatrixXd J,Eigen::MatrixXd Alpha,Eigen::VectorXd x_dot,SVDParameters svd) throw (TPIKMissingDoFInitializationException);
};
#endif
