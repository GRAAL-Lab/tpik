#include "Task.h"
#include <iostream>
#include <rml/RML.h>
#include <eigen3/Eigen/Dense>

namespace tpik {
enum TaskType  {Equality, Inequality};

class CartesianTask : public Task {
public:
    CartesianTask(const std::string ID, int DoF, TaskType taskType);

    ~CartesianTask();

    void SetTaskParameter(TaskParameter taskParameters);

    const TaskParameter& GetTaskParameter();

    void SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShapedParameters);

    const BellShapedParameter& GetIncreasingBellShapedParameter();

    void CheckInitialization() throw (std::exception);

    void SetUseErrorNorm();
protected:
    void ChangeObserver();
    void UpdateInternalActivationFunction() override;
    void UpdateReference() override;
    void SaturateReference();

    Eigen::Vector3d error_;
    Eigen::MatrixXd JObserver_;
    BellShapedParameter increasingBellShape_;
    TaskParameter taskParameter_;
    bool useErrorNorm_{ false };
    bool initializedTaskParameter_{false};
    bool initializedIncreasingBellShapeParameter_{false};
    TaskType taskType_;



};
}
