#ifndef __PRIORITYLEVEL_H__
#define __PRIORITYLEVEL_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>
#include "Task.h"
#include "tpikExceptions.h"


struct SVDParameters
{
	double threshold; 	// the value below which the raised cosine becomes > 0
	double lambda;    	// the maximum value of the raised cosine
	double mu;
	bool flag;
};

class PriorityLevel {
public:
	PriorityLevel(std::string ID);
	PriorityLevel();
	~PriorityLevel();
    void AddTask(std::shared_ptr<Task> task);
    std::string GetID() const throw (PriorityLevelIndexException);
    void SetID(std::string ID);
    void UpdateAll();
    void UpdateJacobian();
    void UpdateInternalActivationFunction();
    void UpdateReference();
    void SetExternalActivationFunction(double Ae);
    void SetSVDParameters(SVDParameters svdParameters);
    const Eigen::MatrixXd& GetJacobian()const;
    Eigen::MatrixXd GetActivationFunction();
    const Eigen::MatrixXd& GetInternalActivationFunction() const;
    double GetExternalActivationFunction();
    const Eigen::VectorXd& GetReference() const;
    int GetNumberOfTask();
    const std::vector<std::shared_ptr<Task> > GetLevel() const;
    SVDParameters GetSVDParameter();
private:
    std::vector<std::shared_ptr<Task> > level_;
	std::string ID_;
    Eigen::MatrixXd Ai_,J_;
    Eigen::VectorXd x_dot_;
    double Ae_;
    int taskNumber_;
    SVDParameters svdParameters_;
};



#endif
