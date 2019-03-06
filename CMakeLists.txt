set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS} )

add_executable(ndt main.cpp)
target_link_libraries(ndt ${PCL_LIBRARIES})