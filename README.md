# DynamicsSim
3-body dynamics sim in C++

# Requirements: 
Ensure [Eigen](https://libeigen.gitlab.io/) is installed for linear algebra computations and [SFML](https://www.sfml-dev.org/) is installed for visualization.

# Design Patterns:

- Builder Pattern: The builder pattern is used in in [SystemFactory.hpp](./include/dynamics/SystemFactory.hpp) to build a dynamics system
- Factory Pattern: The factory pattern is used in [SystemFactory.hpp](./include/dynamics/SystemFactory.hpp) to construct complex scenarios (such as an Earth-Moon system with a spacecraft orbitting the L4 Lagrange point).
- Strategy Pattern: Within a Dynamical system, various [integration algorithms](./include/dynamics/Integrator.hpp) (Forward [Euler](./include/dynamics/Euler.hpp) or [Runge-Kutta](./include/dynamics/RK4.hpp)) and various [ODE](./include/dynamics/ODE.hpp) can be solved (Standard [Gravity Model](./include/dynamics/GravityModel.hpp) or [Circular Restricted 3 Body Problem](./include/dynamics/CR3BP.hpp)).

# Foundational Classes
Most of the code is built off of the [Body](./include/dynamics/Body.hpp) class and [Dynamics](./include/dynamics/Dynamics.hpp) class. Integration methods, ODEs, and visulazation are set up in different classes.

# Core OO Principles
- Inheritance / Polymorphism is demonstrated through the base [ODE](./include/dynamics/ODE.hpp) adn [Integrator](./include/dynamics/Integrator.hpp) classes, inherited by their respective subclasses / algorithms described above. 

- Dependency Injection is demonstrated by passing the [ODE](./include/dynamics/ODE.hpp) adn [Integrator](./include/dynamics/Integrator.hpp) as arguments during construction of the dynamical system.

# Test cases
Minimal testing has been done, and needs to be expanded on, but their are more than 5 basic tests implemented in the [tests](\tests) folder, all running and passing with gtest.