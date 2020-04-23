#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__

#include <ctrl_toolbox/HelperFunctions.h>
#include <eigen3/Eigen/Dense>
#include <libconfig.h++>
#include <memory>

namespace tpik {
/**
 * @brief Parameter used to define a bell shaped function. Used to create either an increasing or a decreasing function with a transitory linear
 * behavior in between xmin and xmax
 */
struct BellShapedParameter {
    Eigen::VectorXd xmin; //!< Vector containing the xmin for all the bell shaped function described by the struct.
    Eigen::VectorXd xmax; //!< Vector containing the xmax for all the bell shaped function described by the struct.
    /**
	 * @brief Overload of the cout operator.
	 */
    friend std::ostream& operator<<(std::ostream& os, BellShapedParameter const& bellShape)
    {
        return os << "\033[1;37m"
                  << "xmin \n"
                  << "\033[0m" << bellShape.xmin << "\n"
                  << "\033[1;37m"
                  << "xmax \n"
                  << "\033[0m" << bellShape.xmax;
    }

    template <typename T>
    void ConfigureFromFile(T& confObj)
    {
        ctb::SetParamVector(confObj, xmin, "xmin");
        ctb::SetParamVector(confObj, xmax, "xmax");
    }
};

/**
 * @brief Task Parameter, used both in the equality and inequality task
 */
struct TaskParameter {
    double gain; //!< The reference gain.
    bool taskEnable; //!< Boolean stating whether the task is active
    double saturation; //!< The reference saturation value.

    /**
	 * @brief Overload of the cout operator.
	 */
    friend std::ostream& operator<<(std::ostream& os, TaskParameter const& taskParam)
    {
        return os << "\033[1;37m"
                  << "gain \n"
                  << "\033[0m" << taskParam.gain << "\n"
                  << "\033[1;37m"
                  << "taskEnable \n"
                  << "\033[0m" << taskParam.taskEnable << "\n"
                  << "\033[1;37m"
                  << "saturation \n"
                  << "\033[0m" << taskParam.saturation;
    }

    template <typename T>
    void ConfigureFromFile(T& confObj)
    {
        ctb::SetParam(confObj, gain, "gain");
        ctb::SetParam(confObj, taskEnable, "enable");
        ctb::SetParam(confObj, saturation, "saturation");
    }
};

/**
 * @brief The TaskType enum to state whether the task is equality or inequality
 */
enum class CartesianTaskType { Equality,
    InequalityDecreasing,
    InequalityIncreasing,
    InequalityInBetween };

/**
 * @brief The InequalityType enum to state wheether an inequality task is increasing decreasing in beteween or none of the aformentioned
 */
enum class InequalityTaskType { Increasing,
    Decreasing,
    Inbetween };

/**
 * @brief The ProjectorType enum definig the projector type
 */
enum class ProjectorType { Default,
    OnPlane,
    OnLine };
}

enum class TaskType {
    Equality,
    InequalityGreaterThan,
    InequalityLessThan,
    InequalityInBetween
};

#endif
