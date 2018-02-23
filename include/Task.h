
#ifndef __TASK_H__
#define __TASK_H__

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iomanip>

namespace tpik{
enum class TaskType{Equality, InequalityLessThan, InequalityGreaterThan, InequalityInBetween}; 
struct BellShapedFunction{
	double sigma1;
	double sigma2;
};

struct TaskParameter{
	BellShapedFunction min,max;
	double gain;
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
	friend std::ostream& operator <<(std::ostream& os, Task const& task){
		return os<< "\033[1;37m"<<"Task ID "<<task.ID_<<"\n"<<std::setprecision(2)
				  << "\033[1;37m"<<"Internal Activation Function \n"<<"\033[0m"<<task.Ai_<<"\n"
				  << "\033[1;37m"<<"Jacobian \n"<<"\033[0m"<<task.J_<<"\n"
				  << "\033[1;37m"<<"Reference \n"<<"\033[0m"<<task.x_dot_<<"\n"
				  << "\033[1;37m"<<"minBound \n"<<"\033[0m"<<task.minBound_<<"\n"
				  << "\033[1;37m"<<"maxBound \n"<<"\033[0m"<<task.maxBound_<<"\n"
				  << "\033[1;37m"<<"TaskParameter (Gain TaskEnable) \n"<<"\033[0m"<<std::setprecision(2)<<task.taskParameter_.gain<<"	"<<task.taskParameter_.TaskEnable<<"\n";};
        
protected:
	TaskType type_;
	std::string ID_;
	double minBound_,maxBound_;
	Eigen::MatrixXd Ai_,J_;
	Eigen::VectorXd x_dot_;
	TaskParameter taskParameter_;

};
}

#endif
