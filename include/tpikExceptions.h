#ifndef __TPIKEXCEPTIONS_H__
#define __TPIKEXCEPTIONS_H__

#include <iostream>
#include <memory>

/**
 * @brief Exception to be thrown when ID has not been initialized
 */

class PriorityLevelIndexException: public std::exception
{
  virtual const char* what() const throw () {
    return "PriorityLevel has not ID, use PriorityLevel::SetID. ";
  }
};

/**
 * @brief Exception to be thrown when Ithe unified heirarchy has not been specified
 */
class ActionManagerHierarchyException: public std::exception
{
  virtual const char* what() const throw () {
    return "Action Manager has not unified hierarchy specified, use SetHierarchy ";
  }
};

/**
 * @brief Exception to be thrown when it is not possible to set the action since it is not present in the
 * actions vector
 */
class ActionManagerNullActionException: public std::exception
{
  virtual const char* what() const throw () {
    return "The action settled is not present in the list of action, use AddAction to add it ";
  }
};

/**
 * @brief Exception to be thrown when Degrees of Freedom have not been initialized
 */
class TPIKMissingDoFInitializationException: public std::exception
{
  virtual const char* what() const throw () {
    return "Degrees of Freedom not specified, use SetDoF";
  }
};

/**
 * @brief Exception to be thrown when Degrees of Freedom have not been initialized
 */
class SolverNotInitializationException: public std::exception
{
  virtual const char* what() const throw () {

	  return "TPIK and/or Action Manager are not initialized, use SetTPIK and/or SetActionManager";
  }
};

#endif
