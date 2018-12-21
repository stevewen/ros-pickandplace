cmake_minimum_required(VERSION 2.8.3)
project(robot_eyes)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    pcl_ros
    pcl_conversions
    sensor_msgs
)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    message_generation
)


add_message_files(FILES
  SegmentedCloud.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  pcl_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES robot_eyes
   CATKIN_DEPENDS message_runtime
   DEPENDS
   roscpp
   rospy
   pcl_ros
   pcl_conversions
   sensor_msgs

)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
add_library(robot_eyes src/nodelets/CloudColorer.cpp)

# add_executable(cloud_colorer src/common/cloud_colorer.cpp)
# add_dependencies(cloud_colorer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_eyes ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(example src/examples/example.cpp)
add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(example_voxelgrid src/examples/example_voxelgrid.cpp)
add_dependencies(example_voxelgrid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_voxelgrid ${catkin_LIBRARIES})

add_executable(simple_plane_segmentation src/examples/simple_plane_segmentation.cpp)
add_dependencies(example_voxelgrid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (simple_plane_segmentation ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(plane_segmenter src/segmentation/plane_segmenter.cpp)
# add_dependencies(plane_segmenter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (plane_segmenter ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(region_growing_segmenter src/segmentation/region_growing_segmenter.cpp)
# add_dependencies(region_growing_segmenter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(region_growing_segmenter  ${catkin_LIBRARIES})

# add_executable(cylinder_segmenter src/segmentation/cylinder_segmenter.cpp)
# add_dependencies(cylinder_segmenter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(cylinder_segmenter ${catkin_LIBRARIES})

# add_executable(euclidian_segmenter src/segmentation/euclidian_segmenter.cpp)
# add_dependencies(euclidian_segmenter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries (euclidian_segmenter ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(plannar_segmenter_ndt src/plannar_segmenter_ndt.cpp)
# add_dependencies(plannar_segmenter_ndt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(plannar_segmenter_ndt ${catkin_LIBRARIES})
