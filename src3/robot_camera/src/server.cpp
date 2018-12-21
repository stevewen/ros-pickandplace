#include <ros/ros.h>
#include "robot_camera/object_struct_sc.h"
#include "std_msgs/String.h"

// 回调函数，输入参数req，输出参数res
bool add(robot_camera::object_struct_sc::Request &req,   robot_camera::object_struct_sc::Response &res)
{
  // res.Class = {"ball1", "ball2", "box", "chips", "magic cube", "blacktea"};
  // res.center_x = { 0.15,  -0.15,   -0.15,   -0.05,   -0.05,   0.05};
  // res.center_y = {-0.1,   -0.05,    0.2,     0.0,    -0.2,    0.12};
  // res.center_z = { 0.55,   0.55,    0.55,    0.5,     0.55,   0.47};

  res.Class = {"box",  "blacktea",  "chips", "ball1", "ball2",  "magic cube"};
  
  res.center_x = {0.0118371, -0.253113, -0.0607576, -0.1550057,  0.085946, 0.130898};
  res.center_y = {0.1448926, 0.054384, -0.010126, 0.123506, 0.05474898, 0.202229};
  res.center_z = {0.484678, 0.530687, 0.530249, 0.495125, 0.568886, 0.404998};

  res.front_x = {-0.471108, -0.0284628, 0.0417648,  0.0,  0.0, 0.259393 };
  res.front_y = {0.560198, -0.804543, 0.780161, 0.0, 0.0, 0.648003};
  res.front_z = {-0.681348, -0.593213, 0.624182, 0.0, 0.0, -0.716105};

  res.plane_a =-0.0287113;
  res.plane_b =-0.757883;
  res.plane_c =-0.651758;
  res.plane_d = 0.441683;

  /*
   box blacktea chips ball1 ball2 magic cube 

   0.0118371 -0.153113 0.0167576 -0.0850057 0.175946 0.130898 
   0.0448926 -0.014384 -0.10126 0.123506 -0.000474898 0.112229 
   0.584678 0.580687 0.630249 0.495125 0.628886 0.504998 

   -0.471108 -0.0284628 0.0417648 0 0 0.259393 
    0.560198 -0.804543 0.780161 0 0 0.648003 
   -0.681348 -0.593213 0.624182 0 0 -0.716105 

   -0.0287113 -0.757883 -0.651758 0.441683

   Quaternion1
    0.908759
   -0.00598859
   -0.0130485
    0.417074

  Quaternion2
   0.908759
  -0.00598859
  -0.0130485
   0.417074

  Quaternion3
   0.908759
  -0.00598859
  -0.0130485
   0.417074

  Quaternion4
   0.908759
  -0.00598859
  -0.0130485
   0.417074

  Quaternion5
   0.908759
  -0.00598859
  -0.0130485
   0.417074

  Quaternion6
   0.908759
  -0.00598859
  -0.0130485
   0.417074

  */

  return true;

}

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "server");

  // 创建节点句柄
  ros::NodeHandle nq;

  // 创建一个server，注册回调函数
  ros::ServiceServer service = nq.advertiseService("object_struct_sc", add);

  // 循环等待回调函数
  ROS_INFO("Please input the target of manipulation");
  ros::spin();

  return 0;
}
