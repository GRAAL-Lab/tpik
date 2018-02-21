#ifndef __PRIORITYLEVEL_H__
#define __PRIORITYLEVEL_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>
#include <memory>
#include "Task.h"
#include "tpikExceptions.h"




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
    void SetSVDParameters(rml::SVDParameters svdParameters);
    const Eigen::MatrixXd& GetJacobian()const;
    Eigen::MatrixXd GetActivationFunction();
    const Eigen::MatrixXd& GetInternalActivationFunction() const;
    double GetExternalActivationFunction();
    const Eigen::VectorXd& GetReference() const;
    int GetNumberOfTask();
    const std::vector<std::shared_ptr<Task> > GetLevel() const;
    rml::SVDParameters GetSVDParameter();
private:
    std::vector<std::shared_ptr<Task> > level_;
	std::string ID_;
    Eigen::MatrixXd Ai_,J_;
    Eigen::VectorXd x_dot_;
    double Ae_;
    int taskNumber_;
    rml::SVDParameters svdParameters_;
};



#endif
