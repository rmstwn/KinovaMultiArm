cmake_minimum_required(VERSION 3.10)

# Project name
project(KinovaMultiArm)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find pkg-config
find_package(PkgConfig REQUIRED)
# Find orocos-kdl
pkg_check_modules(orocos_kdl REQUIRED orocos-kdl)
# Find the orocos_kdl package
find_package(orocos_kdl REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/usr/include/eigen3")
find_package(Eigen3 REQUIRED)

# Find kdl_parser
find_package(kdl_parser)
# Find Boost
find_package(Boost COMPONENTS system filesystem REQUIRED)

# Find the urdfdom package
find_package(urdfdom REQUIRED)

# find OM MODBUS package
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# Find the zeromq package
find_package(cppzmq)

# Set the path to the Kinova Kortex API include directory
set(KORTEX_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/kortex/include")

# Set the path to the Kinova Kortex API library directory
set(KORTEX_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/kortex/lib")

# Add the Kinova Kortex API include directory to the include path
include_directories(${KORTEX_INCLUDE_DIR})
include_directories("${KORTEX_INCLUDE_DIR}/client")
include_directories("${KORTEX_INCLUDE_DIR}/client_stubs")
include_directories("${KORTEX_INCLUDE_DIR}/common")
include_directories("${KORTEX_INCLUDE_DIR}/google")
include_directories("${KORTEX_INCLUDE_DIR}/messages")

# Add the third-party include path
include_directories("${PROJECT_SOURCE_DIR}/thirdParty/cxxopts")

# Add urdfdom include path (add this section)
# include_directories("/usr/include/urdf")
# Alternatively, if installed in a non-standard location:
# include_directories("/usr/local/include/urdf")

# Include the headers
include_directories(${URDFDOM_INCLUDE_DIRS})

add_definitions(-D_OS_UNIX)

# Include orocos-kdl and kdl_parser directories
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

# Include OM MODBUS 
include_directories(${LIBMODBUS_INCLUDE_DIRS})
link_directories(${LIBMODBUS_LIBRARY_DIRS})

# Add the Kinova Kortex API library directory to the library path
link_directories(${KORTEX_LIBRARY_DIR})

# Add the executable
add_executable(KinovaMultiArm 
    src/main.cpp
    src/constants.cpp
    #src/kinova_manager.cpp
    src/fk_vereshchagin.cpp
    src/kdl_eigen_conversions.cpp
    src/oml_mrtu.cpp
    src/KinovaManager.cpp
)

# Link against the Kinova Kortex API library
target_link_libraries(KinovaMultiArm ${KORTEX_LIBRARY_DIR})
target_link_libraries(KinovaMultiArm ${KORTEX_LIBRARY_DIR}/release/libKortexApiCpp.a)

# Link against urdfdom library
target_link_libraries(KinovaMultiArm ${URDFDOM_LIBRARIES})

# Link against orocos_kdl, kdl_parser, Boost libraries
target_link_libraries(KinovaMultiArm
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

# Link against MODBUS libraries
target_link_libraries(KinovaMultiArm ${LIBMODBUS_LIBRARIES})

# Link against zeromq libraries
target_link_libraries(KinovaMultiArm cppzmq)
