cmake_minimum_required(VERSION 3.16)

project(DynamicsSim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
find_package(Eigen3 REQUIRED)
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()

# ---- Libraries ----

add_library(dynamics
    src/dynamics/Body.cpp
    src/dynamics/Dynamics.cpp
    src/dynamics/SystemFacade.cpp
    src/dynamics/GravityModel.cpp
    src/dynamics/RK4.cpp
    src/dynamics/Euler.cpp
    src/dynamics/CR3BP.cpp
    src/dynamics/control/NoControl.cpp
    src/dynamics/control/SimpleControl.cpp
    src/dynamics/control/EnsembleControl.cpp
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

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
    tests/*.cpp
)

add_executable(tests
    ${TEST_SOURCES}
)

target_link_libraries(tests
    dynamics   # your library
    gtest
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(tests)

# ---- Executable ----

add_executable(simulator
    apps/simulator.cpp
)

target_link_libraries(simulator
    dynamics
    vis
)