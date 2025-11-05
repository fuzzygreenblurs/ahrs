cmake_minimum_required(VERSION 3.14)       
project(fc_slam VERSION 1.0.0 LANGUAGES CXX) 
set(CMAKE_CXX_STANDARD 17)               
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)            

add_library(ahrs src/madgwick.cpp)
target_include_directories(ahrs PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ahrs Eigen3::Eigen)

pkg_check_modules(CPPUTEST REQUIRED cpputest)
add_executable(test_ahrs test/test_ahrs.cpp)
target_include_directories(test_ahrs PRIVATE ${CPPUTEST_INCLUDE_DIRS})
target_link_libraries(test_ahrs PRIVATE ahrs ${CPPUTEST_LDFLAGS})


