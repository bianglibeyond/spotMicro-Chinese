cmake_minimum_required(VERSION 3.5)

# Set the project name
project (spot_micro_kinematics)

set(CMAKE_CXX_STANDARD 14)

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

find_package(Eigen3 REQUIRED NO_MODULE)

# Add an library for the example classes
add_library(spot_micro_kinematics 
    src/spot_micro_kinematics.cpp
	src/utils.cpp
	src/spot_micro_leg.cpp
)

target_include_directories(spot_micro_kinematics
	 PUBLIC 
		${PROJECT_SOURCE_DIR}/include
)

#target_link_libraries(spot_micro_kinematics
	#Eigen3::Eigen
#)

#############################################
# Unit tests
# Only add 3rd party directory if gtest doesnt already exist
if (NOT TARGET gtest)
		add_subdirectory(3rd_party/google-test)
endif ()
# enable CTest testing
enable_testing()

# Add a testing executable
add_executable(test_utils tests/test_utils.cpp)

target_link_libraries(test_utils
    spot_micro_kinematics
    gtest 
    gtest_main
)

add_test(test_utils test_utils)


# Add a testing executable
add_executable(test_spot_micro_leg tests/test_spot_micro_leg.cpp)

target_link_libraries(test_spot_micro_leg
    spot_micro_kinematics
    gtest 
    gtest_main
)

add_test(test_spot_micro_leg test_spot_micro_leg)

# Add a testing executable
add_executable(test_spot_micro_kinematics tests/test_spot_micro_kinematics.cpp)

target_link_libraries(test_spot_micro_kinematics
    spot_micro_kinematics
    gtest 
    gtest_main
)

add_test(test_spot_micro_kinematics test_spot_micro_kinematics)
