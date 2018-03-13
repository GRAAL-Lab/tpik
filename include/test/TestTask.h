#ifndef __TESTTASK_H__
#define __TESTTASK_H__

#include <tpik/TPIKlib.h>
#include <iostream>

class TestTask: public tpik::InequalityTask
{
public:
	TestTask(const std::string ID);
	TestTask();
	~TestTask();
	void SetID(const std::string ID);
	void Update() override;
	void SetGain(std::shared_ptr<Eigen::MatrixXd> gain);
protected:
	void UpdateInternalActivationFunction() override;
	void UpdateJacobian() override;
	void UpdateReference() override;

private:
	std::shared_ptr<Eigen::MatrixXd> gain_;
};

#endif

