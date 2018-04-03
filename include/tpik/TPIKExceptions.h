#ifndef __TPIKEXCEPTIONS_H__
#define __TPIKEXCEPTIONS_H__

#include <iostream>

namespace tpik
{

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

class ActionManagerMissingPriorityLevelException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "The PL level does not exist in the unified hierarchy, use the methods AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithRegularization";
	}
};
/**
 * @brief Exception to be thrown when an Action priority level is not present in the unified hierarchy.
 */

class ActionManagerMissingActionPriorityLevelException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "One of the Action PL level does not exist in the unified hierarchy, use the methods AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithSVD";
	}
};

/**
 * @brief Exception to be thrown when it is not possible to set the action since it is not present in the
 * actions vector.
 */
class ActionManagerNullActionException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "The action is not present in the list of action, use AddAction to add it ";
	}
};

/**
 * @brief Exception to be thrown when taskParameter have not been initialized .
 */

class TaskParameterNotInitializedException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "Task Parameter not initialized, use SetTaskParameter ";
	}
};
/**
 * @brief Exception to be thrown when minBound have not been initialized .
 */

class MinBoundNotInitializedException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "Minimum Bound has not initialized, use SetMinBound ";
	}
};

/**
 * @brief Exception to be thrown when maxBound have not been initialized .
 */

class MaxBoundNotInitializedException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "Maximum Bound has not initialized, use SetMinBound ";
	}
};
/**
 * @brief Exception to be thrown when bellShapedParameter have not been initialized .
 */

class BellShapeParametersNotInitializedException: public std::exception
{
	virtual const char* what() const throw ()
	{
		return "BellShapeParameters has not initialized, use SetBellShapeParameters ";
	}
};
}

#endif
