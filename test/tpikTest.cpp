#include <tpik/TPIKlib.h>
#include <rml/RML.h>
#include <iostream>
#include <memory>
#include "test/TestTask.h"

int main()
{
	std::cout << "Testing Solver ..." << std::endl;
	const std::string ID1 = "ID1";
	const std::string ID2 = "ID2";
	const std::string IDPL1 = "IDPL1";
	const std::string IDPL2 = "IDPL2";
	const std::string IDAction1 = "act1";
	const std::string IDAction2 = "act2";
	const std::string IDAction3 = "act3";
	int taskSpace = 6;
	int DoF = 6;

	rml::SVDData svd;
	svd.params.lambda = 0.00;
	svd.params.threshold = 0.00;
	auto testTask1 = std::make_shared<TestTask>(TestTask(ID1));
	auto testTask2 = std::make_shared<TestTask>(TestTask(ID2));
	auto gain1 = std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(taskSpace, DoF));
	//Setting Gain
	testTask1->SetGain(gain1);
	testTask1->Update();
	//TASK 2
	auto gain2 = std::make_shared<Eigen::MatrixXd>(Eigen::MatrixXd::Identity(taskSpace, DoF));
	testTask2->SetGain(gain2);
	testTask2->Update();
	//Action Manager Defintion
	auto actionManager = std::make_shared<tpik::ActionManager>(tpik::ActionManager());
	actionManager->AddPriorityLevelToHierarchyWithSVD(IDPL1, svd);
	actionManager->AddTaskToPriorityLevel(testTask1, IDPL1);
	actionManager->AddTaskToPriorityLevel(testTask2, IDPL1);
	actionManager->AddPriorityLevelToHierarchyWithSVD(IDPL2, svd);
	actionManager->AddTaskToPriorityLevel(testTask1, IDPL2);
	actionManager->AddTaskToPriorityLevel(testTask2, IDPL2);
	actionManager->AddAction(IDAction1, std::vector<std::string> { IDPL1 });
	actionManager->AddAction(IDAction2, std::vector<std::string> { IDPL1 });
	actionManager->AddAction(IDAction3, std::vector<std::string> { IDPL2 });
	//SOLVER AND TPIK TRIAL
	auto iCat = std::make_shared<tpik::iCAT>(tpik::iCAT(6));
	auto solver = std::make_shared<tpik::Solver>(tpik::Solver(actionManager, iCat));
	std::cout << *solver << std::endl;
	//TEST CHANGING ACTION
	std::cout << "************Testing changing of action**********" << std::endl;
	//WITH ACTION ACT 3
	std::cout << "ACTION: act3" << std::endl;
	solver->SetAction(IDAction3);
	Eigen::VectorXd y = solver->ComputeVelocities();
	std::cout << *solver << std::endl;
	std::cout << "COMPUTED VELOCITY" << std::endl;
	std::cout << "ACTION: act1" << std::endl;
	solver->SetAction(IDAction1);
	std::cout << *solver << std::endl;
	y = solver->ComputeVelocities();
	std::cout << "COMPUTED VELOCITY" << std::endl;
	//TEST CHANGING PARAMETER OF JACOBIAN
	std::cout << "************Testing changing of jacobian**********" << std::endl;
	testTask1->SetGain(gain1);
	testTask2->SetGain(gain2);
	*gain1 = 0.01 * Eigen::MatrixXd::Identity(6, 6);
	*gain2 = 0.02 * Eigen::MatrixXd::Identity(6, 6);
	testTask1->SetGain(gain1);
	testTask2->SetGain(gain2);
	testTask1->Update();
	testTask2->Update();
	y = solver->ComputeVelocities();
	std::cout << "Computed vel" << std::endl;
	*gain1 = 0.1 * Eigen::MatrixXd::Identity(6, 6);
	*gain2 = 0.1 * Eigen::MatrixXd::Identity(6, 6);
	testTask1->Update();
	testTask2->Update();
	y = solver->ComputeVelocities();
	std::cout << "SOLVER\n" << *solver << std::endl;
	std::cout << "DESIRED VELOCITY \n" << y << std::endl;

	return 0;
}
