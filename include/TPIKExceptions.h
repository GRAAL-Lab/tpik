#ifndef __TPIKEXCEPTIONS_H__
#define __TPIKEXCEPTIONS_H__

#include <iostream>
#include <memory>

namespace tpik {
/**
 * @brief Exception to be thrown when priority level ID has not been initialized.
 */

class PriorityLevelIndexException: public std::exception {
	virtual const char* what() const throw () {
		return "PriorityLevel has not ID, use PriorityLevel::SetID. ";
	}
};

/**
 * @brief Exception to be thrown when the unified hierarchy has not been specified.
 */
class ActionManagerHierarchyException: public std::exception {
	virtual const char* what() const throw () {
		return "Action Manager has not unified hierarchy specified, use AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithSVD ";
	}
};
/**
 * @brief Exception to be thrown when a priority Level is not present in the unified hierarchy.
 */
//TODO add the pl ID
class ActionManagerMissingPriorityLevel: public std::exception {
	virtual const char* what() const throw () {
		return "The PL level does not exist in the unified hierarchy, use AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithSVD";
	}
};
/**
 * @brief Exception to be thrown when an Action priority level is not present in the unified hierarchy.
 */

//TODO add the pl ID
class ActionManagerMissingActionPriorityLevel: public std::exception {
	virtual const char* what() const throw () {
		return "One of the Action PL level does not exist in the unified hierarchy, use AddPriorityLevelToHierarchy or AddPriorityLevelToHierarchyWithSVD";
	}
};

/**
 * @brief Exception to be thrown when it is not possible to set the action since it is not present in the
 * actions vector.
 */
//TODO add action ID
class ActionManagerNullActionException: public std::exception {
	virtual const char* what() const throw () {
		return "The action settled is not present in the list of action, use AddAction to add it ";
	}
};

/**
 * @brief Exception to be thrown when the degrees of freedom have not been initialized.
 */
class TPIKMissingDoFInitializationException: public std::exception {
	virtual const char* what() const throw () {
		return "Degrees of Freedom not specified, use SetDoF";
	}
};

/**
 * @brief Exception to be thrown when either the tpik or the action manager have not been initialized
 */
class SolverNotInitializationException: public std::exception {
	virtual const char* what() const throw () {

		return "TPIK and/or Action Manager are not initialized, use SetTPIK and/or SetActionManager";
	}
};
}

#endif
