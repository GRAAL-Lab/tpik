![CI](https://github.com/GRAAL-Lab/tpik/actions/workflows/ci.yml/badge.svg)

# TPIK: Task Priority Inverse Kinematic Library

Implementation in C++ of the Task Priority Inverse Kinematic Algorithm. The library can be found in the [GRAAL Robotics Toolbox](https://github.com/topics/graal-robotics-toolbox) repositories.

## Documentation

Full doxygen-generated documentation can be found at: [TPIK Documentation](https://www.graal.dibris.unige.it/docs/tpik/).

## Description

This library implements the task priority inverse kinematic thanks to the definition of the abstract class `tpik::Task`. The class deriving form the `tpik::Task` base class must implement the pure virtual methods in order to compute the task Jacobian, Activation Function and Reference.
  
The abstract classes `tpik::ReactiveTask`, `tpik::NonReactiveTask` are also provided. In such classes the task is described by more parameters and some virtual methods are implemented.
 
Thanks to the `tpik::ActionManager` it is possible to gather tasks sharing the same priority into priority levels, define a unified hierarchy which contains all the priority level ordered by priority and define a set of actions that the reference scenario requires.

The Inequality Constraints Activation and Task algorithm to compute a single step is implemented in the `tpik::iCAT::ComputeVelocities` class. 
 
Once the tasks and the different required structures are defined (by using the action manager), thanks to the `tpik::Solver` class it is possible to compute the kinematic control in the priority framework by taking into account also the transiction in between different actions.

## Dependencies

Before building the repository you will have to install the following dependencies:

* Eigen 3: `sudo apt install libeigen3-dev`
* libconfig++ : `sudo apt install libconfig++-dev`
* rml: `git clone https://github.com/GRAAL-Lab/rml.git`

## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

```bash
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

## Usage Example

The following example shows the typical TPIK workflow:

1. create one or more tasks,
2. define how each task updates its Jacobian, activation, and reference,
3. assign tasks to priority levels,
4. group priority levels into actions,
5. create a solver and compute joint velocities for the selected action.

In this example, `TestTask` is a simple reactive task derived from `tpik::ReactiveTask`. Its `Update()` method refreshes:

* the internal activation matrix,
* the Jacobian,
* and the task reference.

In particular, the reference is set inside `UpdateReference()` of the `TestTask` through:

```cpp
x_dot_bar_ = Eigen::VectorXd::Ones(6);
```

so each time `Update()` is called, the task target velocity reference is updated accordingly. An example usage of this class is the following:


```cpp
#include "test/TestTask.h"
#include <iostream>
#include <memory>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

int main()
{
    const std::string ID1 = "ID1";
    const std::string ID2 = "ID2";
    const std::string IDPL1 = "IDPL1";
    const std::string IDPL2 = "IDPL2";
    const std::string IDAction1 = "act1";
    const std::string IDAction2 = "act2";
    const std::string IDAction3 = "act3";

    const int taskSpace = 6;
    const int DoF = 6;

    // Regularization parameters for the priority levels
    rml::RegularizationData regularizationData;
    regularizationData.params.lambda = 0.001;
    regularizationData.params.threshold = 0.0001;

    // Create two example tasks
    auto task1 = std::make_shared<TestTask>(ID1);
    auto task2 = std::make_shared<TestTask>(ID2);

    // Set task gains
    auto gain1 = std::make_shared<Eigen::MatrixXd>(
        Eigen::MatrixXd::Identity(taskSpace, DoF));
    auto gain2 = std::make_shared<Eigen::MatrixXd>(
        Eigen::MatrixXd::Identity(taskSpace, DoF));

    task1->SetGain(gain1);
    task2->SetGain(gain2);

    // Update each task: activation, Jacobian, and reference are refreshed here
    task1->Update();
    task2->Update();

    // Create the ActionManager and define priority levels
    auto actionManager = std::make_shared<tpik::ActionManager>();

    actionManager->AddPriorityLevelWithRegularization(IDPL1, regularizationData);
    actionManager->AddTaskToPriorityLevel(task1, IDPL1);
    actionManager->AddTaskToPriorityLevel(task2, IDPL1);

    actionManager->AddPriorityLevelWithRegularization(IDPL2, regularizationData);
    actionManager->AddTaskToPriorityLevel(task1, IDPL2);
    actionManager->AddTaskToPriorityLevel(task2, IDPL2);

    // Define the global hierarchy
    actionManager->SetUnifiedHierarchy({IDPL1, IDPL2});

    // Define actions as subsets of the hierarchy
    actionManager->AddAction(IDAction1, {IDPL1});
    actionManager->AddAction(IDAction2, {IDPL1});
    actionManager->AddAction(IDAction3, {IDPL2});

    // Create the inverse kinematics backend
    auto iCat = std::make_shared<tpik::iCAT>(6);

    Eigen::VectorXd satMin(6), satMax(6);
    satMin.setZero();
    satMax.setOnes();
    iCat->SetSaturation(satMin, satMax);

    // Create the solver
    auto solver = std::make_shared<tpik::Solver>(actionManager, iCat);

    // Activate an action and compute the resulting joint velocities
    solver->SetAction(IDAction3, true);
    Eigen::VectorXd dq = solver->ComputeVelocities();

    std::cout << "Computed velocities for " << IDAction3 << ":\n"
              << dq.transpose() << std::endl;

    // Switch action at runtime
    solver->SetAction(IDAction1, true);
    dq = solver->ComputeVelocities();

    std::cout << "Computed velocities for " << IDAction1 << ":\n"
              << dq.transpose() << std::endl;

    return 0;
}
```


This example creates two reactive tasks and updates them before solving. In `TestTask`, the Jacobian is built from the assigned gain matrix, while the task reference is explicitly set in `UpdateReference()` through `x_dot_bar_`. This means that the desired task-space behavior is not passed directly in `main()`, but encapsulated inside the task implementation itself.

The tasks are then assigned to two priority levels, and the priority levels are grouped into actions. The solver can switch between actions at runtime with `SetAction(...)`, making it possible to reuse the same hierarchy structure while enabling different control modes.

### Key points

* `Update()` should be called whenever task data changes.
* The task reference is defined inside the task class through `UpdateReference()`.
* The Jacobian can also be task-specific and is updated in `UpdateJacobian()`.
* `ActionManager` organizes tasks into priority levels and actions.
* `Solver` computes joint velocities from the currently active action.


## Citation

If you find this project useful, please consider citing:

```latex
@article{simetti2018task,
  title={Task priority control of underwater intervention systems: Theory and applications},
  author={Simetti, E and Casalino, G and Wanderlingh, F and Aicardi, M},
  journal={Ocean Engineering},
  volume={164},
  pages={40--54},
  year={2018},
  publisher={Elsevier}
}
```

### License

The software is released under the MIT License, as reported in the [LICENSE.md](/LICENSE.md) file.

### Maintainer

This project is mantained by the [GRAAL Laboratory](https://www.graal.dibris.unige.it/), University of Genoa (Italy).
