# DynamicsSim
3-body dynamics sim in C++

# Requirements: 
Ensure [Eigen](https://libeigen.gitlab.io/) is installed for linear algebra computations and [SFML](https://www.sfml-dev.org/) is installed for visualization. CMake / C++ must also be used for building, and [gcov](https://gcc.gnu.org/onlinedocs/gcc/Gcov.html) / a working python installation are required for test coverage.

# Building:
To build this project, after cloning and entering the repo run:

```bash
mkdir build
cd build
cmake ..
# cmake -DENABLE_COVERAGE=ON .. #run this to build with code coverage
make -j 4
./simulator
# ./tests #To run unit tests
```

# Design Patterns:

- Builder Pattern: The builder pattern is used in in [SystemFactory.hpp](./include/dynamics/SystemFactory.hpp) to build a dynamics system
- Factory Pattern: The factory pattern is used in [SystemFactory.hpp](./include/dynamics/SystemFactory.hpp) to construct complex scenarios (such as an Earth-Moon system with a spacecraft orbitting the L4 Lagrange point).
- Strategy Pattern: Within a Dynamical system, various [integration algorithms](./include/dynamics/Integrator.hpp) (Forward [Euler](./include/dynamics/Euler.hpp) or [Runge-Kutta](./include/dynamics/RK4.hpp)) and various [ODE](./include/dynamics/ODE.hpp) can be solved (Standard [Gravity Model](./include/dynamics/GravityModel.hpp) or [Circular Restricted 3 Body Problem](./include/dynamics/CR3BP.hpp)).
- MVC Pattern: For visualization, the MVC pattern is used with a wrapper [Renderer](./include/vis/Renderer.hpp) class included. The Model is the simulation as a whole. The [View](./include/vis/view.hpp) is primarily in charge of rendering circles and trails using the SFML library. The [Controller](./include/vis/Controller.hpp) is in charge of resizing the frame if necessary, keeping track of the spacecraft trail, and determining object sizes for rendering.

# Foundational Classes
Most of the code is built off of the [Body](./include/dynamics/Body.hpp) class and [Dynamics](./include/dynamics/Dynamics.hpp) class. Integration methods, ODEs, and visulazation are set up in different classes.

# Core OO Principles
- Inheritance / Polymorphism is demonstrated through the base [ODE](./include/dynamics/ODE.hpp) and [Integrator](./include/dynamics/Integrator.hpp) classes, inherited by their respective subclasses / algorithms described above. 

- Dependency Injection is demonstrated by passing the [ODE](./include/dynamics/ODE.hpp) and [Integrator](./include/dynamics/Integrator.hpp) as arguments during construction of the dynamical system, rather than creating instances of them within the class.

# Test cases
Test cases providing 100% coverage of the dynamics package is included. To run testing, build with testing enabled and run the provided gtests. Following this, run the following command to generate a coverage report using gcov:

```bash
gcovr -r ..   --filter '.*src/.*\.cpp$'   --exclude '.*_deps.*'   --exclude '.*googletest.*'   --exclude '.*gtest.*'   --exclude '.*tests.*'   --html --html-details   -o coverage.html
```