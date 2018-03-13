#include "test/TestTask.h"
#include <iostream>
#include <rml/RML.h>

TestTask::TestTask(const std::string ID) :
		tpik::InequalityTask(ID, 6, 6)
{
	ID_ = "TestTask" + ID_;

}

TestTask::~TestTask()
{
}

void TestTask::SetID(const std::string ID)
{
	ID_ = "TestTask" + ID;
}

void TestTask::UpdateInternalActivationFunction()
{

	Ai_ = Eigen::MatrixXd::Identity(6, 6);
}

void TestTask::UpdateJacobian()
{
	J_ = *gain_ * (Eigen::MatrixXd::Identity(6, 6));
}

void TestTask::UpdateReference()
{

	x_dot_ = Eigen::VectorXd::Ones(6);
}

void TestTask::SetGain(std::shared_ptr<Eigen::MatrixXd> gain)
{
	gain_ = gain;
}

void TestTask::Update()
{

	UpdateInternalActivationFunction();
	UpdateJacobian();
	UpdateReference();
}
