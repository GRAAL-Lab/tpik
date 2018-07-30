#ifndef __INEQUALITYTASK_H__
#define __INEQUALITYTASK_H__

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "TPIKDefines.h"
#include "Task.h"
#include "TPIKExceptions.h"

namespace tpik
{
/**
 * @brief InequalityTask class, derived from the Abstract class Task.\n
 * Implementation of the inequality task. The derived classes must implement the pure virtual methods UpdateActivationFunction,
 * UpdateJacobian and UpdateReference.\n
 * Such class is provided with method to set and get the increasing bell shaped parameter and the decreasing bell shaped parameter in order to state the interval
 * where the task is active.\n
 * Furthermore methods to set and get the task parameter (gain, reference saturation and a boolean stating whether the task is enabled) are implemented.
 */
class InequalityTask: public Task
{
public:
	/**
	 * @brief Constructor of Task Class.\n
	 * Initialization of the class variables:\n
	 * Jacobian (taskSpace x DoF)\n
	 * Internal Activation Function (taskSpace x taskSpace)\n
	 * Reference (taskSpace x 1)\n
	 * @param[in] ID Task ID.
	 * @param[in] taskSpace;
	 * @param[in] DoF;
	 * @param[in] bellShapeDecreasingUsed : boolean stating whether the decreasing bell shaped parameter are used and needed to be set;
	 * @parma[in] bellShapedIncreasingUsed : boolean stating whether the increasing bell shaped parameter are used and needed to be set.
	 */
	InequalityTask(const std::string ID, int taskSpace, int DoF, bool bellShapeDecreasingUsed, bool BellShapedIncreasingUsed);
	/**
	 * @brief Default De-constructor of InequalityTask Class.
	 */
	virtual ~InequalityTask();
	/**
	 * @brief Method that sets the Task Parameter (gain, task enable boolean).
	 * @param[in] taskParameters tpik::TaskParameter struct.
	 */
	void SetTaskParameter(TaskParameter taskParameters);
	/**
	 * @brief Method that returns the TaskEquality Parameter
	 * @returns TaskParameter
	 */
	const TaskParameter& GetTaskParameter();
	/**
	 * @brief Method increasing bell shaped parameters.
	 * @param[in] increasingBellShapedParameters tpik::BellShapedParameter for increasing curve.
	 */
	void SetIncreasingBellShapedParameter(BellShapedParameter increasingBellShapedParameters);
	/**
	 * @brief Method settings decreasing bell shaped parameter.
	 * @param[in] decreasingBellShapedParameters tpik::BellShapedParameter for decreasing curve.
	 */
	void SetDecreasingBellShapedParameter(BellShapedParameter decreasingBellShapedParameters);
	/**
	 * @brief Method returning the task increasing bell shaped parameter.
	 * @returns tpik::BellShapedParameter for increasing curve.
	 */
	const BellShapedParameter& GetIncreasingBellShapedParameter();
	/**
	 * @brief Method returning the task decreasing bell shaped parameter.
	 * @returns tpik::BellShapedParameter for decreasing curve.
	 */
	const BellShapedParameter& GetDecreasingBellShapedParameter();
	/**
	 * @brief Method which chek whether all the needed variables have been initialized.
	 * @note An exception is thrown if either the bellShapedParameter or the Task parameter have not been initialized yet.
	 */
    void CheckInitialization() throw (ExceptionWithHow);
	/**
	 * Method stating whether the increasing bell shape parameter are used.
	 * @return true if the bell shape increasing parameter are used, false otherwise.
	 */
	bool GetBellShapeIncreasingUsed();
	/**
	 * Method stating whether the decreasing bell shape parameter are used.
	 * @return true if the bell shape decreasing parameter are used, false otherwise.
	 */
	bool GetBellShapeDecreasingUsed();
	/**
	 * @brief Overload of the cout operator.
	 */
	friend std::ostream& operator <<(std::ostream& os, InequalityTask const& inequality)
	{
		os << "\033[1;37m" << "INEQUALITY TASK : "<< inequality.ID_ << "\n"<<std::setprecision(4)
				<< "Internal Activation Function \n" << "\033[0m" << inequality.Ai_ << "\n"
				<< "\033[1;37m" << "Jacobian \n" << "\033[0m" << inequality.J_ << "\n"
				<< "\033[1;37m" << "Reference \n" << "\033[0m" << inequality.x_dot_ <<"\n"
				<< "\033[0m"<< inequality.taskParameter_ ;

	if (inequality.bellShapeDecreasingUsed_) {
		os <<  "\033[1;37m" << "decreasing bell shape\n" << "\033[0m" << inequality.decreasingBellShape_ <<"\n";


	}
	if (inequality.bellShapeIncreasingUsed_) {
		os <<  "\033[1;37m" << "increasing bell shape\n" << "\033[0m" << inequality.increasingBellShape_ <<"\n";
	}
		return os;
	}

protected:
	/**
	 * @brief Method saturating the reference using the member variable saturation of the taskParameter struct.
	 */
	void SaturateReference();
	TaskParameter taskParameter_; //!< The tpik::TaskParameter.
	bool initializedTaskParameter_{false}; //!< The boolean stating whether the tpik::TaskParameter struct has been initialized.
	bool initializedDecreasingBellShapeParameter_{false}; //!< The boolean stating whether the tpik::BellShapedParameter struct for increasing curve has been initialized.
	bool initializedIncreasingBellShapeParameter_{false}; //!< The boolean stating whether the tpik::BellShapedParameter struct for decreasing curve has been initialized.
	bool bellShapeIncreasingUsed_{false}; //!< The  boolean stating whether the increasing tpik::BellShapedParameter are used. To be defined in the constructor of the derived classes.
	bool bellShapeDecreasingUsed_{false}; //!< The  boolean stating whether the decreasing tpik::BellShapedParameter are used. To be defined in the constructor of the derived classes
	BellShapedParameter increasingBellShape_; //!< The tpik::BellShapedParameter for increasing curve.
	BellShapedParameter decreasingBellShape_; //!< The tpik::BellShapedParameter for decreasing curve.

};
}

#endif
