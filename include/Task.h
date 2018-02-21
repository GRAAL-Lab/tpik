
#ifndef __TASK_H__
#define __TASK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

enum class TaskType{Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween}; 
struct BellShapedFunction{
	double sigma1;
	double sigma2;
};

struct TaskParameter{
	BellShapedFunction min,max;
	bool TaskEnable;
};

class Task {
public:
    Task(TaskType type,const std::string ID); // ID is set by the user in order to uniquely identify the task
	Task(TaskType type);
    virtual ~Task();
    void SetID(const std::string ID);
	void SetMinBound(double minBound);
	void SetMaxBound(double  maxBound);
	const Eigen::MatrixXd& GetJacobian() const;// in this way from the main it will be not possible to change both the pointer and the matrix
	const Eigen::MatrixXd& GetInternalActivationFunction() const;
	const Eigen::VectorXd& GetReference() const;
	void SetTaskParameter(TaskParameter taskParameter);
	virtual void UpdateInternalActivationFunction()=0;
	virtual void UpdateReference()=0;
	virtual void UpdateJacobian()=0;
	void UpdateAll();
        
protected:
	TaskType type_;
	std::string ID_;
	double minBound_,maxBound_;
	Eigen::MatrixXd Ai_,J_;
	Eigen::VectorXd x_dot_;
	TaskParameter taskParameter_;

};

#endif
