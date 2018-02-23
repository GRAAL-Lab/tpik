#ifndef __PRIORITYLEVEL_H__
#define __PRIORITYLEVEL_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rml/RML.h>
#include <memory>
#include "Task.h"
#include "TPIKExceptions.h"



namespace tpik{
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
    friend std::ostream& operator <<(std::ostream& os, PriorityLevel const& priorityLevel){
    		return os << "\033[1;37m" << "PriorityLevel ID " << priorityLevel.ID_ << "\n"<<std::setprecision(2)
    				  << "\033[1;37m" << "Internal Activation Function \n" << "\033[0m" << priorityLevel.Ai_ << "\n"
    				  << "\033[1;37m" << "External Activation Function " << "\033[0m" << priorityLevel.Ae_ << "\n"
    				  << "\033[1;37m" << "Jacobian \n" << "\033[0m" << priorityLevel.J_ << "\n"
    				  << "\033[1;37m" << "Reference \n" << "\033[0m" << priorityLevel.x_dot_ << "\n"
    				  << "\033[1;37m" << "svdParameters\nThrehsold " << "\033[0m" << priorityLevel.svdParameters_.threshold << "\n"
					  << "\033[1;37m" << "lambda " << "\033[0m" << priorityLevel.svdParameters_.lambda << "\n"
					  << "\033[1;37m" << "mu " << "\033[0m" << priorityLevel.svdParameters_.mu << "\n" << "\033[0m";};

private:
    std::vector<std::shared_ptr<Task> > level_;
	std::string ID_;
    Eigen::MatrixXd Ai_, J_;
    Eigen::VectorXd x_dot_;
    double Ae_;
    int taskNumber_;
    rml::SVDParameters svdParameters_;
};
}



#endif
