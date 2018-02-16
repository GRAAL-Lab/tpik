#ifndef __PRIORITYLEVEL_H__
#define __PRIORITYLEVEL_H__


#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "Task.h"


struct SVDParameters
{
	double threshold; 	// the value below which the raised cosine becomes > 0
	double lambda;    	// the maximum value of the raised cosine
	double mu;
};

class PriorityLevel {
public:
	PriorityLevel(std::string ID);
	~PriorityLevel();
    void AddTask(std::shared_ptr<Task> task);
    std::string GetID();
    void UpdateAll();
    void UpdateJacobian();
    void UpdateActivationFunction();
    void UpdateReference();
    void SetExternalActivationFunction(Eigen::MatrixXd Ae);
    void SetSVDParameters(SVDParameters svdParameters);
    const Eigen::MatrixXd& GetJacobian()const;
    const Eigen::MatrixXd& GetActivationFunction() const;
    const Eigen::MatrixXd& GetInternalActivationFunction() const;
    const Eigen::MatrixXd& GetExternalActivationFunction() const;
    const Eigen::VectorXd& GetReference() const;
    int GetNumberOfTask();
    const std::vector<std::shared_ptr<Task> > GetLevel() const;
private:
    std::vector<std::shared_ptr<Task> > level_;
	std::string ID_;
    Eigen::MatrixXd Ai_,Ae_,J_,A_;
    Eigen::VectorXd x_dot_;
    int taskNumber_;
    SVDParameters svdParameters_;
};
#endif
