cmake_minimum_required(VERSION 3.21.2)
project(simCommander VERSION 0.1.0)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# Find required packages
find_package(Protobuf 3.9.1 REQUIRED)
find_package(Qt5 COMPONENTS Network REQUIRED)

# Compile all proto files and create grSimCommanderProto target
add_subdirectory(proto)

# Create target that contains all the necessary includes
add_subdirectory(include)

# Create library
add_subdirectory(src)

# Create example executables
add_subdirectory(examples)
