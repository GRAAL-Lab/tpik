/*!\mainpage Documentation Index
 *
 * \section intro_sec Introduction
 *
 * This is the doxygen documentation for the tpik (task priority inverse kinematic) library.
 * \section Dependencies

Before building the repository you will have to install the following dependencies:

    - Eigen 3 (sudo apt install libeigen3-dev)
    - rml (https://bitbucket.org/isme_robotics/rml)

 * \section Description
 * This library implements the task priority inverse kinematic thanks to the definition of the abstract class Task. The class deriving form the Task base Class must implement the pure virtual methods in order to compute the task Jacobian, Activation Function and Reference.
  
 *The abstract classes InequalityTask and EqualityTask are also provided. In such classes the task is described by more parameters but the pure virtual methods to compute the Jacobian, Activation function and Reference must still be implemented.
 
 * Thanks to the ActionManager it is possible to gather tasks sharing the same priority in priority levels, define a unified hierarchy which contains all the priority level ordered by priority and define a set of actions that the reference scenario requires.
 
  *In order to compute the inverse kinematic control for a single priority level one must implement the pure virtual method ComputeYSingleLevel() of the TPIK class. The Inequality Constraints Activation and Task algorithm to compute a single step is implemented in the iCAT class. 
   
*Once the tasks and the different required structures are defined (by using the action manager), Thanks to the Solver class it is possible to compute the kinematic control in the priority framework by taking into account also the transiction in between different actions.   

* \section Build 
The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install
*/
