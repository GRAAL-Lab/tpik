/*!\mainpage Documentation Index
 *
 * \section intro_sec Introduction
 *
 * This is the doxygen documentation for the tpik (task priority inverse kinematic) library.
 *
 * \section descr_sec Description
 * This library implements the task priority inverse kinematic thanks to the definition of the abstract class Task. The class deriving form the Task base Class must implement the pure virtual methods in order to compute the task Jacobian, Activation Function and Reference. 
 
*Tasks which share the same priority are manipulated via the PriorityLevel Task. 
Once all the task are defined and gathered depending on their priority level, the user must define the different control Action via the Action and Action Manager class, as well as the unified hierarchy containing all the priorityLevel ordered by priority.

*In order to compute the inverse kinematic control for a single priority level one must implement the pure virtual method ComputeYStep of the TPIK class. The Inequality Constraints Activation and Task algorithm to compute a single step is implemented in the iCAT class. 

*By using the Solver Class, it is possible to compute the velocity by setting the current action and updating the jacobian, Activation function and reference of the Derived Task classes.  

*In order to implement the iCat algorithm, juxtaposing the matrices in the PriorityLevel class and compute the Activation Function the rml library has been used.

*https://bitbucket.org/isme_robotics/rml

*/
