#ifndef __TPIKDEFINES_H__
#define __TPIKDEFINES_H__

//#include <ctrl_toolbox/HelperFunctions.h>
#include <eigen3/Eigen/Dense>
#include <libconfig.h++>
#include <memory>
#include "ConfHelpers.h"

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
    bool ConfigureFromFile(T& confObj) noexcept(false)
    {
        if (!GetParamVector(confObj, xmin, "xmin")) {
            return false;
        }
        if (!GetParamVector(confObj, xmax, "xmax")) {
            return false;
        }

        return true;
    }
};

/**
 * @brief Task Parameter, used both in the equality and inequality task
 */
struct TaskParameter {
    bool taskEnable = false; //!< Boolean stating whether the task is active
    
    double gain = 0.0; //!< The reference gain used in calculation.
    double conf_gain = 0.0;  //!< A backup variable for reference gain loaded from the configuration file.

    double saturation = 0.0; //!< The reference saturation value.
    double conf_saturation = 0.0;  //!< A backup variable for reference saturation loaded from the configuration file.

    /**
	 * @brief Overload of the cout operator.
	 */
    friend std::ostream& operator<<(std::ostream& os, TaskParameter const& taskParam)
    {
        return os << "\033[1;37m"
                  << "taskEnable \n"
                  << "\033[0m" << taskParam.taskEnable << "\n"<< "\033[1;37m"
                  << "gain \n"
                  << "\033[0m" << taskParam.gain << "\n"
                  << "\033[1;37m"
                  << "saturation \n"
                  << "\033[0m" << taskParam.saturation;
    }

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        if (!GetParam(confObj, taskEnable, "enable")) {
            return false;
        }
    
        if (!GetParam(confObj, gain, "gain")) {
            return false;
        }
        conf_gain = gain; // Backup value in case of online value changes        

        if (!GetParam(confObj, saturation, "saturation")) {
            return false;
        }
        conf_saturation = saturation; // Backup value in case of online value changes
        
        return true;
    }
};
/**
 * @brief The ProjectorType enum definig the projector type
 */
enum class ProjectorType {
    Default,
    OnLine,
    OnPlane,
};

enum class TaskType {
    Equality,
    Inequality
};

enum class TaskOption {
    Default,
    UseErrorNorm,
    ActiveOnNorm,
};
}

#endif
