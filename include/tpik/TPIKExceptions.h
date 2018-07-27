#ifndef __TPIKEXCEPTIONS_H__
#define __TPIKEXCEPTIONS_H__

#include <iostream>

namespace tpik
{

class ExceptionWithID: public std::exception
{
public:

	void SetID(std::string ID)
	{
		ID_ = ID;
	}
	const char* who() const throw ()
	{
		return ID_.c_str();
	}
private:
	std::string ID_;
};

/**
 * @brief Exception to be thrown when the unified hierarchy has not been specified.
 */
class ActionManagerHierarchyException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "Action Manager has not unified hierarchy specified, use AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithSVD ";
	}

};

/**
 * @brief Exception to be thrown when a priority Level is not present in the unified hierarchy.
 */

class ActionManagerMissingPriorityLevelException: public ExceptionWithID
{

	virtual const char* what() const throw ()
	{
		return "The PL level does not exist in the unified hierarchy, use the methods AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithRegularization";
	}

};

/**
 * @brief Exception to be thrown when it is not possible to set the action since it is not present in the
 * actions vector.
 */
class ActionManagerNullActionException: public ExceptionWithID
{
	virtual const char* what() const throw ()
	{
		return "The action is not present in the list of action, use AddAction to add it ";
	}
};

/**
 * @brief Exception to be thrown when taskParameter have not been initialized .
 */

class TaskParameterNotInitializedException: public ExceptionWithID
{
	virtual const char* what() const throw ()
	{
		return "Task Parameter not initialized, use SetTaskParameter ";
	}
};
/**
 * @brief Exception to be thrown when minBound have not been initialized .
 */
class BellShapedDecreasingNotInitializedException: public ExceptionWithID
{
	virtual const char* what() const throw ()
	{
		return "Decreasing bell shaped parameter not initialized, use SetDecreasingBellShapedParameter ";
	}
};

/**
 * @brief Exception to be thrown when maxBound have not been initialized .
 */

class BellShapedIncreasingNotInitializedException: public ExceptionWithID
{
	virtual const char* what() const throw ()
	{
		return "Increasing bell shaped parameter not initialized, use SetIncreasingBellShapedParameter ";
	}
};

/**
 * @brief Exception to be thrown when the transformation matrix wTg has not been initialized.
 */

class WrongBellShapeParameterSizeException: public tpik::ExceptionWithID
{
    virtual const char* what() const throw ()
    {
        return "wrong size bell shape parameters";
    }
};
}

#endif
