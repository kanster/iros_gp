cmake_minimum_required(VERSION 2.8)

project("data_partition")


# find point cloud library
find_package(PCL 1.8.0 REQUIRED)
if(PCL_FOUND)
    message(STATUS "PCL INCLUDE DIRS ${PCL_INCLUDE_DIRS}")
    message(STATUS "PCL LIBS REQUIRED ${PCL_LIBRARIES}")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()


find_package(Boost REQUIRED thread system filesystem)

find_package(VTK REQUIRED)
if(VTK_FOUND)
    message(STATUS "VTK INCLUDE DIRS ${VTK_INCLUDE_DIRS}")
    message(STATUS "VTK LIBS REQUIRED ${VTK_LIBRARIES}")
    include_directories(${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARY_DIRS})
endif()

add_executable(${PROJECT_NAME} main.cpp )
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})

add_executable(partition partition.cpp )
target_link_libraries(partition ${PCL_LIBRARIES} )
