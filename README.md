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

The typical TPIK workflow is the following:

  1. define one or more tasks,
  2. assign them to priority levels,
  3. group priority levels into actions,
  4. create a solver with an inverse kinematics backend,
  5. select an action and compute joint velocities.

The following example code creates two tasks, assigns them to two priority levels, and then defines three actions that enable different subsets of the hierarchy. A `tpik::Solver` is then instantiated with an `iCAT` backend and used to compute velocity commands for the currently selected action. The active action can be changed at runtime with `SetAction(...)`, allowing the controller to switch behavior without rebuilding the whole hierarchy.

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

    auto gain1 = std::make_shared<Eigen::MatrixXd>(
        Eigen::MatrixXd::Identity(taskSpace, DoF));
    auto gain2 = std::make_shared<Eigen::MatrixXd>(
        Eigen::MatrixXd::Identity(taskSpace, DoF));

    task1->SetGain(gain1);
    task2->SetGain(gain2);

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

### Key classes

* `TestTask`: example task implementation
* `tpik::ActionManager`: manages tasks, priority levels, and actions
* `tpik::iCAT`: inverse kinematics backend
* `tpik::Solver`: computes the output velocities from the active action

### Notes

* Call `Update()` on tasks after changing gains, Jacobians, references, or internal task data.
* Use `SetUnifiedHierarchy(...)` to define the global task-priority structure.
* Use `AddAction(...)` to expose reusable control modes built from selected priority levels.
* Saturation limits can be configured on the IK backend before solving.


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
