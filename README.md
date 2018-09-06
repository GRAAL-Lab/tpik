# TPIKlib: Task Priority Inverse Kinematic Library

## Description
This library implements the task priority inverse kinematic thanks to the definition of the abstract class tpik::Task. The class deriving form the tpik::Task base Class must implement the pure virtual methods in order to compute the task Jacobian, Activation Function and Reference.
  
The abstract classes tpik::InequalityTask, tpik::EqualityTask and tpik::CartesianTask are also provided. In such classes the task is described by more parameters and some virtual methods are implemented.
 
Thanks to the tpik::ActionManager it is possible to gather tasks sharing the same priority into priority levels, define a unified hierarchy which contains all the priority level ordered by priority and define a set of actions that the reference scenario requires.

In order to compute the inverse kinematic control for a single priority level one must implement the pure virtual method `ComputeYSingleLevel()` of the tpik::TPIK class. The Inequality Constraints Activation and Task algorithm to compute a single step is implemented in the tpik::iCAT class. 
 
Once the tasks and the different required structures are defined (by using the action manager), thanks to the tpik::Solver class it is possible to compute the kinematic control in the priority framework by taking into account also the transiction in between different actions.

## Dependencies
Before building the repository you will have to install the following dependencies:

* Eigen 3 (`sudo apt install libeigen3-dev`)
* rml ( 'https://merosss.bitbucket.io/rml/' )

## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install
