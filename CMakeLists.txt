cmake_minimum_required(VERSION 3.16)

project(DynamicsSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Libraries ----

add_library(dynamics
    src/dynamics/Body.cpp
    src/dynamics/Dynamics.cpp
    src/dynamics/GravityModel.cpp
    src/dynamics/RK4.cpp
    src/dynamics/Euler.cpp
)

target_link_libraries(dynamics PUBLIC Eigen3::Eigen)
target_include_directories(dynamics PUBLIC include)


add_library(vis
    src/vis/renderer.cpp
)

target_link_libraries(vis
    PUBLIC
    SFML::Graphics
    Eigen3::Eigen
)

target_include_directories(vis PUBLIC include)

# ---- Executable ----

add_executable(simulator
    apps/simulator.cpp
)

target_link_libraries(simulator
    dynamics
    vis
)