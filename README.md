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
