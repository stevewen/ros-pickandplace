/*********************************************************************
 * Software License Agreement (BSD License)
 *********************************************************************
 Author:       Alejandro Acevedo
 Modified by:  Daniel Ordonez    10/May/2018
 Modified by:  Wen lin           28/Dec/2018
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "robot_camera/object_struct_sc.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//For add object
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/mesh_operations.h"

// Logging capabilities
#include <ros/console.h>
#include <math.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace Eigen;

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "robot_pick_and_place");

  // 创建节点句柄
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 创建一个client，请求service
  ros::ServiceClient client = node_handle.serviceClient<robot_camera::object_struct_sc>("object_struct_sc");

  // 创建service消息
  robot_camera::object_struct_sc srv;
  srv.request.ifstart = "ON";

  // 发布service请求，等待应答结果
  if (client.call(srv))
  {

      int num = srv.response.Class.size();
      float res_x[num],res_y[num],res_z[num]; 
      float ori_x[num],ori_y[num],ori_z[num];
      std::string res_name[num];
            
      for (int i = 0; i < num; i++)
      {
 
        float  cxx = 0.0;  float  cyy = 0.0;  float  czz = 0.0;

        if(srv.response.Class[i].compare("box") == 0)
        {
        res_name[0] = srv.response.Class[i];
        res_x[0] = srv.response.center_x[i] + cxx;
        res_y[0] = srv.response.center_y[i] + cyy;
        res_z[0] = srv.response.center_z[i] + czz;
        ori_x[0] = srv.response.front_x[i];
        ori_y[0] = srv.response.front_y[i];
        ori_z[0] = srv.response.front_z[i];
        }

        if(srv.response.Class[i].compare("chips") == 0)
        {
        res_name[1] = srv.response.Class[i];
        res_x[1] = srv.response.center_x[i] + cxx;
        res_y[1] = srv.response.center_y[i] + cyy;
        res_z[1] = srv.response.center_z[i] + czz;
        ori_x[1] = srv.response.front_x[i];
        ori_y[1] = srv.response.front_y[i];
        ori_z[1] = srv.response.front_z[i];
        }

        if(srv.response.Class[i].compare("ball1") == 0)
        {
        res_name[2] = srv.response.Class[i];
        res_x[2] = srv.response.center_x[i] + cxx;
        res_y[2] = srv.response.center_y[i] + cyy;
        res_z[2] = srv.response.center_z[i] + czz;
        ori_x[2] = srv.response.front_x[i];
        ori_y[2] = srv.response.front_y[i];
        ori_z[2] = srv.response.front_z[i];
        }

        if(srv.response.Class[i].compare("ball2") == 0)
        {
        res_name[3] = srv.response.Class[i];
        res_x[3] = srv.response.center_x[i] + cxx;
        res_y[3] = srv.response.center_y[i] + cyy;
        res_z[3] = srv.response.center_z[i] + czz;
        ori_x[3] = srv.response.front_x[i];
        ori_y[3] = srv.response.front_y[i];
        ori_z[3] = srv.response.front_z[i];
        }
        
        if(srv.response.Class[i].compare("magic cube") == 0)
        {
        res_name[4] = srv.response.Class[i];
        res_x[4] = srv.response.center_x[i] + cxx;
        res_y[4] = srv.response.center_y[i] + cyy;
        res_z[4] = srv.response.center_z[i] + czz;
        ori_x[4] = srv.response.front_x[i];
        ori_y[4] = srv.response.front_y[i];
        ori_z[4] = srv.response.front_z[i];
        }

        if(srv.response.Class[i].compare("blacktea") == 0)
        {
        res_name[5] = srv.response.Class[i];
        res_x[5] = srv.response.center_x[i] + cxx;
        res_y[5] = srv.response.center_y[i] + cyy;
        res_z[5] = srv.response.center_z[i] + czz;
        ori_x[5] = srv.response.front_x[i];
        ori_y[5] = srv.response.front_y[i];
        ori_z[5] = srv.response.front_z[i];
        }

        ROS_INFO("Get the target: %f", srv.response.center_z[i]);
      }

  // ROS_INFO_STREAM("Get the target, preparing to grasp " << inputString);
  
  // Change console log level to DEBUG. (Optional)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Read Ros parameter indicating whther we are operating a simulation or the real robot
  // Variable used for detecting whether a motion plan was succesful.
  bool success;

  // Set up move group objects
  moveit::planning_interface::MoveGroupInterface right_gripper_move_group("right_gripper");
  right_gripper_move_group.allowReplanning(true);     // Allow move group to re plan motions when scene changes are detected
  moveit::planning_interface::MoveGroupInterface left_gripper_move_group("left_gripper");
  left_gripper_move_group.allowReplanning(true);     // Allow move group to re plan motions when scene changes are detected
  moveit::planning_interface::MoveGroupInterface arm_right_move_group("arm_right");
  arm_right_move_group.allowReplanning(true);     // Allow move group to re plan motions when scene changes are detected
  moveit::planning_interface::MoveGroupInterface arm_left_move_group("arm_left");
  arm_left_move_group.allowReplanning(true);     // Allow move group to re plan motions when scene changes are detected
  moveit::planning_interface::MoveGroupInterface csda10f_move_group("csda10f");
  csda10f_move_group.allowReplanning(true);      // Allow move group to re plan motions when scene changes are detected
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *csda10f_joint_model_group = csda10f_move_group.getCurrentState()->getJointModelGroup("csda10f");
  const robot_state::JointModelGroup *arm_left_joint_model_group = arm_left_move_group.getCurrentState()->getJointModelGroup("arm_left");
  const robot_state::JointModelGroup *arm_right_joint_model_group = arm_right_move_group.getCurrentState()->getJointModelGroup("arm_right");

  // Holder for motion plans.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // RobotState instance that will hold move group robot states instances.
  robot_state::RobotState start_state(*csda10f_move_group.getCurrentState());

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Define visualization parameters
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("camera_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // Set up tutorial text position
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().y() = 0.50; // above head of CSDA10F

  visual_tools.publishText(text_pose, "Robot Pick and Place Demo \n Press next to start", rvt::WHITE, rvt::XXLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");


// *****************************************************************************************
// STEP 1: Go to home position.

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
  csda10f_move_group.setNamedTarget("home_arms_folded");
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go Back Home Position", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  csda10f_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

// *****************************************************************************************
// STEP 2: Add manipulation object.
  visual_tools.publishText(text_pose, "Loading mesh model...", rvt::WHITE, rvt::XXLARGE);
  // Creating Environment
  // Create vector to hold 6 collision objects.

 // 0**********************************************************************************
  //Vector to scale
  Vector3d vectorScale(1.8, 1.8, 1.8);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object;
  // The id of the object is used to identify it.
  object.id = res_name[0];
  object.header.frame_id = "camera_link";
  //Path where is located le model
  shapes::Mesh* m = shapes::createMeshFromResource("package://robot_planning/meshes/cubetoy.stl", vectorScale);
  // Define and load the mesh
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  object.meshes.resize(1);
  object.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh_pose;
  mesh_pose.position.x = res_x[0];
  mesh_pose.position.y = res_y[0];
  mesh_pose.position.z = res_z[0];
  // mesh_pose.orientation.x = 0.0;
  // mesh_pose.orientation.y = 0.0;
  // mesh_pose.orientation.z = 0.0;
  // mesh_pose.orientation.w = 1.0;
  tf2::Quaternion ori;
  ori.setRPY(0.0, 0.0 , 0.0);
  mesh_pose.orientation.x = ori.x();
  mesh_pose.orientation.y = ori.y();
  mesh_pose.orientation.z = ori.z();
  mesh_pose.orientation.w = ori.w();

  //The object is added like un colission object
  object.meshes.push_back(mesh);
  object.mesh_poses.push_back(mesh_pose);
  object.operation = object.ADD;      
  // operations are be object.REMOVE, object.APPEND, object.MOVE

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(object);

  // Add the collision objects into the world
  moveit_msgs::ObjectColor itcolor;
  itcolor.id = "Snow";
  itcolor.color.r = 250;
  itcolor.color.g = 250;
  itcolor.color.b = 250;
  itcolor.color.a = 1;
  std::vector<moveit_msgs::ObjectColor> it_color;
  it_color.push_back(itcolor);

  planning_scene_interface.addCollisionObjects(objects,it_color);

 // 1**********************************************************************************
  //Vector to scale
  // Vector3d vectorScale1(0.0012, 0.0012, 0.0025);
  Vector3d vectorScale1(0.001, 0.001, 0.002);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object1;
  // The id of the object is used to identify it.
  object1.id = res_name[1];
  object1.header.frame_id = "camera_link";
  //Path where is located le model
  shapes::Mesh* m1 = shapes::createMeshFromResource("package://robot_planning/meshes/glass.stl", vectorScale1);
  // Define and load the mesh
  shape_msgs::Mesh mesh1;
  shapes::ShapeMsg mesh1_msg;
  shapes::constructMsgFromShape(m1, mesh1_msg);
  mesh1 = boost::get<shape_msgs::Mesh>(mesh1_msg);

  object1.meshes.resize(1);
  object1.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh1_pose;
  mesh1_pose.position.x = res_x[1];
  mesh1_pose.position.y = res_y[1];
  mesh1_pose.position.z = res_z[1];
  // mesh1_pose.orientation.x = 0.0;
  // mesh1_pose.orientation.y = 0.0;
  // mesh1_pose.orientation.z = 0.0;
  // mesh1_pose.orientation.w = 1.0;
  tf2::Quaternion ori1;
  ori1.setRPY(0.0, 0.0 , 0.0);
  mesh1_pose.orientation.x = ori1.x();
  mesh1_pose.orientation.y = ori1.y();
  mesh1_pose.orientation.z = ori1.z();
  mesh1_pose.orientation.w = ori1.w();

  //The object is added like un colission object
  object1.meshes.push_back(mesh1);
  object1.mesh_poses.push_back(mesh1_pose);
  object1.operation = object.ADD;      
  // operations are be object.REMOVE, object.APPEND, object.MOVE

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects1;
  objects1.push_back(object1);

  // Add the collision objects into the world
  moveit_msgs::ObjectColor itcolor1;
  itcolor1.id = "Snow";
  itcolor1.color.r = 250;
  itcolor1.color.g = 250;
  itcolor1.color.b = 250;
  itcolor1.color.a = 1;
  std::vector<moveit_msgs::ObjectColor> it_color1;
  it_color1.push_back(itcolor1);

  planning_scene_interface.addCollisionObjects(objects1,it_color1);

 // 3**********************************************************************************
  //Vector to scale
  Vector3d vectorScale3(0.04, 0.04, 0.04);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object3;
  // The id of the object is used to identify it.
  object3.id = res_name[2];
  object3.header.frame_id = "camera_link";
  //Path where is located le model
  shapes::Mesh* m3 = shapes::createMeshFromResource("package://robot_planning/meshes/ball2.obj", vectorScale3);
  // Define and load the mesh
  shape_msgs::Mesh mesh3;
  shapes::ShapeMsg mesh3_msg;
  shapes::constructMsgFromShape(m3, mesh3_msg);
  mesh3 = boost::get<shape_msgs::Mesh>(mesh3_msg);

  object3.meshes.resize(1);
  object3.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh3_pose;
  mesh3_pose.position.x = res_x[2];
  mesh3_pose.position.y = res_y[2];
  mesh3_pose.position.z = res_z[2];
  // mesh3_pose.orientation.x = 0.0;
  // mesh3_pose.orientation.y = 0.0;
  // mesh3_pose.orientation.z = 0.0;
  // mesh3_pose.orientation.w = 1.0;
  tf2::Quaternion ori3;
  ori3.setRPY(0.0, 0.0 , 0.0);
  mesh3_pose.orientation.x = ori3.x();
  mesh3_pose.orientation.y = ori3.y();
  mesh3_pose.orientation.z = ori3.z();
  mesh3_pose.orientation.w = ori3.w();

  //The object is added like un colission object
  object3.meshes.push_back(mesh3);
  object3.mesh_poses.push_back(mesh3_pose);
  object3.operation = object.ADD;      
  // operations are be object.REMOVE, object.APPEND, object.MOVE

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects3;
  objects3.push_back(object3);

  // Add the collision objects into the world
  moveit_msgs::ObjectColor itcolor3;
  itcolor3.id = "Snow";
  itcolor3.color.r = 250;
  itcolor3.color.g = 250;
  itcolor3.color.b = 250;
  itcolor3.color.a = 1;
  std::vector<moveit_msgs::ObjectColor> it_color3;
  it_color3.push_back(itcolor3);

  planning_scene_interface.addCollisionObjects(objects3,it_color3);

 // 2**********************************************************************************
  //Vector to scale
  Vector3d vectorScale2(0.0005, 0.0005, 0.0005);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object2;
  // The id of the object is used to identify it.
  object2.id = res_name[3];
  object2.header.frame_id = "camera_link";
  //Path where is located le model
  shapes::Mesh* m2 = shapes::createMeshFromResource("package://robot_planning/meshes/ball1.obj", vectorScale2);
  // Define and load the mesh
  shape_msgs::Mesh mesh2;
  shapes::ShapeMsg mesh2_msg;
  shapes::constructMsgFromShape(m2, mesh2_msg);
  mesh2 = boost::get<shape_msgs::Mesh>(mesh2_msg);

  object2.meshes.resize(1);
  object2.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh2_pose;
  mesh2_pose.position.x = res_x[3];
  mesh2_pose.position.y = res_y[3];
  mesh2_pose.position.z = res_z[3];
  // mesh2_pose.orientation.x = 0.0;
  // mesh2_pose.orientation.y = 0.0;
  // mesh2_pose.orientation.z = 0.0;
  // mesh2_pose.orientation.w = 1.0;
  tf2::Quaternion ori2;
  ori2.setRPY(0.0, 0.0 , 0.0);
  mesh2_pose.orientation.x = ori2.x();
  mesh2_pose.orientation.y = ori2.y();
  mesh2_pose.orientation.z = ori2.z();
  mesh2_pose.orientation.w = ori2.w();

  //The object is added like un colission object
  object2.meshes.push_back(mesh2);
  object2.mesh_poses.push_back(mesh2_pose);
  object2.operation = object.ADD;      
  // operations are be object.REMOVE, object.APPEND, object.MOVE

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects2;
  objects2.push_back(object2);

  // Add the collision objects into the world
  moveit_msgs::ObjectColor itcolor2;
  itcolor2.id = "Snow";
  itcolor2.color.r = 250;
  itcolor2.color.g = 250;
  itcolor2.color.b = 250;
  itcolor2.color.a = 1;
  std::vector<moveit_msgs::ObjectColor> it_color2;
  it_color2.push_back(itcolor2);

  planning_scene_interface.addCollisionObjects(objects2,it_color2);

 // 4**********************************************************************************
  //Vector to scale
  Vector3d vectorScale4(0.003, 0.003, 0.003);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object4;
  // The id of the object is used to identify it.
  object4.id = res_name[4];
  object4.header.frame_id = "camera_link";
  //Path where is located le model
  shapes::Mesh* m4 = shapes::createMeshFromResource("package://robot_planning/meshes/cube.obj", vectorScale4);
  // Define and load the mesh
  shape_msgs::Mesh mesh4;
  shapes::ShapeMsg mesh4_msg;
  shapes::constructMsgFromShape(m4, mesh4_msg);
  mesh4 = boost::get<shape_msgs::Mesh>(mesh4_msg);

  object4.meshes.resize(1);
  object4.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh4_pose;
  mesh4_pose.position.x = res_x[4];
  mesh4_pose.position.y = res_y[4];
  mesh4_pose.position.z = res_z[4];
  // mesh4_pose.orientation.x = 0.0;
  // mesh4_pose.orientation.y = 0.0;
  // mesh4_pose.orientation.z = 0.0;
  // mesh4_pose.orientation.w = 1.0;
  tf2::Quaternion ori4;
  ori4.setRPY(0.0, 0.0 , 0.0);
  mesh4_pose.orientation.x = ori4.x();
  mesh4_pose.orientation.y = ori4.y();
  mesh4_pose.orientation.z = ori4.z();
  mesh4_pose.orientation.w = ori4.w();

  //The object is added like un colission object
  object4.meshes.push_back(mesh4);
  object4.mesh_poses.push_back(mesh4_pose);
  object4.operation = object.ADD;      
  // operations are be object.REMOVE, object.APPEND, object.MOVE

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects4;
  objects4.push_back(object4);

  // Add the collision objects into the world
  moveit_msgs::ObjectColor itcolor4;
  itcolor4.id = "Snow";
  itcolor4.color.r = 250;
  itcolor4.color.g = 250;
  itcolor4.color.b = 250;
  itcolor4.color.a = 1;
  std::vector<moveit_msgs::ObjectColor> it_color4;
  it_color4.push_back(itcolor4);

  planning_scene_interface.addCollisionObjects(objects4,it_color4);

 // 5**********************************************************************************
  //Vector to scale
  Vector3d vectorScale5(0.022, 0.022, 0.012);
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject object5;
  // The id of the object is used to identify it.
  object5.id = res_name[5];
  object5.header.frame_id = "camera_link";
  //Path where is located le model
  shapes::Mesh* m5 = shapes::createMeshFromResource("package://robot_planning/meshes/bottle.obj", vectorScale5);
  // Define and load the mesh
  shape_msgs::Mesh mesh5;
  shapes::ShapeMsg mesh5_msg;
  shapes::constructMsgFromShape(m5, mesh5_msg);
  mesh5 = boost::get<shape_msgs::Mesh>(mesh5_msg);

  object5.meshes.resize(1);
  object5.mesh_poses.resize(1);

  //Define a pose for the object (specified relative to frame_id)
  geometry_msgs::Pose mesh5_pose;
  mesh5_pose.position.x = res_x[5];
  mesh5_pose.position.y = res_y[5];
  mesh5_pose.position.z = res_z[5];
  // mesh5_pose.orientation.x = 0.0;
  // mesh5_pose.orientation.y = 0.0;
  // mesh5_pose.orientation.z = 0.0;
  // mesh5_pose.orientation.w = 1.0;
  tf2::Quaternion ori5;
  ori5.setRPY(0.0, 0.0 , 0.0);
  mesh5_pose.orientation.x = ori5.x();
  mesh5_pose.orientation.y = ori5.y();
  mesh5_pose.orientation.z = ori5.z();
  mesh5_pose.orientation.w = ori5.w();

  //The object is added like un colission object
  object5.meshes.push_back(mesh5);
  object5.mesh_poses.push_back(mesh5_pose);
  object5.operation = object.ADD;      
  // operations are be object.REMOVE, object.APPEND, object.MOVE

  // Create vector of collision object messages for the planning_scene_interface
  std::vector<moveit_msgs::CollisionObject> objects5;
  objects5.push_back(object5);

  // Add the collision objects into the world
  moveit_msgs::ObjectColor itcolor5;
  itcolor5.id = "Snow";
  itcolor5.color.r = 250;
  itcolor5.color.g = 250;
  itcolor5.color.b = 250;
  itcolor5.color.a = 1;
  std::vector<moveit_msgs::ObjectColor> it_color5;
  it_color5.push_back(itcolor5);

  planning_scene_interface.addCollisionObjects(objects5,it_color5);

// ***********************************************************************

// ***********************************************************************
  ros::Duration(1.0).sleep();

for (int ip = 0; ip < 3; ip++)
  { 

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object added into the world \n Press next and input the target of pick", rvt::CYAN, rvt::XXLARGE); 
  visual_tools.trigger(); 
  visual_tools.prompt("Please choose the target within 'box' 'cube' 'ball' 'balloon' 'chipbox' and 'teabox' \n");
  // 从终端命令行获取抓取目标
  std::cin.clear(); 
  std::string inputString;
  std::getline(std::cin, inputString);
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  moveit_msgs::CollisionObject objectx;
  geometry_msgs::Pose meshx_pose;
  std::string graspx_right_gripper;
  float dyy;
  if (inputString == "box")
  {
      objectx = object;  dyy=0.02;
      meshx_pose = mesh_pose;
      graspx_right_gripper= "grasp1_right_gripper";
  }
  else if (inputString == "cube")
  {
      objectx = object4; dyy=0.02;
      meshx_pose = mesh4_pose;
      graspx_right_gripper= "grasp_right_gripper";
  }
  else if (inputString == "ball")
  {
      objectx = object2; dyy=0.02;
      meshx_pose = mesh2_pose;
      graspx_right_gripper= "grasp_right_gripper";
  }
  else if (inputString == "balloon")
  {
      objectx = object3; dyy=0.0;
      meshx_pose = mesh3_pose;
      graspx_right_gripper= "grasp1_right_gripper";
  }
  else if (inputString == "chipbox")
  {
      objectx = object1; dyy=0.02;
      meshx_pose = mesh1_pose;
      graspx_right_gripper= "grasp2_right_gripper";
  }
  else if(inputString == "teabox")
  {
      objectx = object5; dyy=0.02;
      meshx_pose = mesh5_pose;
      graspx_right_gripper= "grasp2_right_gripper";
  }
  else {
  ROS_ERROR("Your choose is NOT correct!");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Press next and input the target of pick AGAIN!!", rvt::CYAN, rvt::XXLARGE); 
  visual_tools.trigger(); 
  visual_tools.prompt("Please choose the target within 'box' 'cube' 'ball' 'balloon' 'chipbox' and 'teabox' \n");
  // 从终端命令行获取抓取目标
  std::cin.clear(); 
  std::string inputString;
  std::getline(std::cin, inputString);
  }
  
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Get the taget, preparing to grasp \n Press next to pick ", rvt::BLUE, rvt::XXLARGE);
  visual_tools.trigger(); 
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // ***********************************************************************************************
  // STEP 3: APPROACH CUP

  // Set up target pose for right arm.
  geometry_msgs::Pose approach_pose = meshx_pose;    // Use target object pose (x,y,z) coordinates as reference.
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI/2 , M_PI , 0.0);
  approach_pose.orientation.x = orientation.x();
  approach_pose.orientation.y = orientation.y();
  approach_pose.orientation.z = orientation.z();
  approach_pose.orientation.w = orientation.w();
  approach_pose.position.z += 0.02;                 // Move 8 cm above the target object object.
  approach_pose.position.x += 0.02;
  approach_pose.position.y += dyy;  

  arm_right_move_group.setStartState(*arm_right_move_group.getCurrentState());  // 
  arm_right_move_group.setPoseTarget(approach_pose);
  // Compute motion plan
  success = (arm_right_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pick Tutorial: picking movements %s", success ? "SUCCESS" : "FAILED");

  // Visualizing plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approach_pose, "Picking Position");
  visual_tools.publishText(text_pose, "Planning for object grasping \n Press next to perform motion on the real robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_right_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Once user allow it, exceute the planned trajectory
  ROS_INFO("Performing motion on real robot...");
  arm_right_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

  //************************************************************************************************
  // STEP 3.1: The right gripper grasp.

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  right_gripper_move_group.setStartState(*right_gripper_move_group.getCurrentState());
  right_gripper_move_group.setNamedTarget(graspx_right_gripper);  // "grasp_right_gripper"
  success = (right_gripper_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.publishText(text_pose, "The right gripper grasp", rvt::WHITE, rvt::XXLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  right_gripper_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

//************************************************************************************************
// STEP 4: ATTACH THE OBJECT TO THE GRIPPER

  moveit::planning_interface::MoveGroupInterface right_gripper_mg("right_gripper");
  right_gripper_mg.attachObject(objectx.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object attached to the gripper \n Press next to place", rvt::PURPLE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Sleep to allow MoveGroup to recieve and process the attached collision object message
  // ros::Duration(1.0).sleep();

//************************************************************************************************
// STEP 5: PLACE THE OBJECT

  visual_tools.publishText(text_pose, "Placing Object inside the drum...\n (this might take a few seconds)", rvt::WHITE, rvt::XXLARGE);

  // Set drop position *******************************
  geometry_msgs::Pose drop_pose = approach_pose;
  tf2::Quaternion orient;
  orient.setRPY(M_PI/2 , 0.0 , M_PI/3);
  drop_pose.orientation = tf2::toMsg(orient);
  // drop_pose.orientation.x = orient.x();
  // drop_pose.orientation.y = orient.y();
  // drop_pose.orientation.z = orient.z();
  // drop_pose.orientation.w = orient.w();

  drop_pose.position.x =  0.7;     //            //[meters]
  drop_pose.position.y = -0.82;    //            //[meters]
  drop_pose.position.z =  0.88;    //            //[meters]
  csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
  csda10f_move_group.setPoseTarget(drop_pose, "arm_right_link_tcp");

  // This movement will involve the manipulation of the 15 joints at the same time, and will be a long sweep, that is why we give some seconds to the solver
  // to find a solution...
  csda10f_move_group.setPlanningTime(20.0);
  csda10f_move_group.setMaxAccelerationScalingFactor(0.5);
  csda10f_move_group.setMaxVelocityScalingFactor(0.2);
  csda10f_move_group.setNumPlanningAttempts(3);
  //Even do it will take time replanning improves results... we are not in a hurry.

  // Plan the motion with constraints.
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // If no solution is found for dropping the object into the drum, roll back to home position and try again.
  if(!success){
    ROS_DEBUG("Planning from table to drum was not possible, going back to home position for replanning");
    // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
    csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
    csda10f_move_group.setNamedTarget("home_arms_folded");
    success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Planning to drop position not feasible \n Comming back to home position for replanning...", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
    // Perform motion on real robot
    csda10f_move_group.execute(my_plan);
    ros::Duration(1.5).sleep();
    // Replan to target position
    ROS_DEBUG("Replanning to target position...");
    csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
    csda10f_move_group.setPoseTarget(drop_pose, "arm_right_link_tcp");
    success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  if(!success){
    ROS_ERROR("No planning found to target position ... :c");
    return 1;
  }
  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approach_pose, "start");
  visual_tools.publishAxisLabeled(drop_pose, "goal");
  visual_tools.publishText(text_pose, "Perform motion of the cup mantaining orientation to avoid liquid spilling \n Press next to perform motion on real robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Once user allow it, exceute the planned trajectory
  ROS_INFO("Performing motion on real robot...");
  csda10f_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  right_gripper_mg.detachObject(objectx.id);   // o

  // When done with the path constraint be sure to clear it.
  csda10f_move_group.clearPathConstraints();

  //************************************************************************************************
  // STEP 5.1: The right gripper open.

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  right_gripper_move_group.setStartState(*right_gripper_move_group.getCurrentState());
  right_gripper_move_group.setNamedTarget("open_right_gripper");
  success = (right_gripper_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // visual_tools.publishText(text_pose, "The right gripper open", rvt::WHITE, rvt::XXLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  visual_tools.deleteAllMarkers();
  right_gripper_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

  // *****************************************************************************************
  // STEP 6: PLAN TO PRE TEACH HOME POSITION

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
  csda10f_move_group.setNamedTarget("home_arms_folded");
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go back to home position \n Press next to perform motion", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  visual_tools.deleteAllMarkers();
  csda10f_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

// *****************************************************************************************
//
//
// *****************************************************************************************
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object added into the world \n Press next and input the target of pick", rvt::CYAN, rvt::XXLARGE); 
  visual_tools.trigger(); 
  visual_tools.prompt("Please choose the target within 'box' 'cube' 'ball' 'balloon' 'chipbox' and 'teabox' \n");
  // 从终端命令行获取抓取目标
  std::cin.clear(); 
  std::string inputString2;
  std::getline(std::cin, inputString2);
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  moveit_msgs::CollisionObject objecty;
  geometry_msgs::Pose meshy_pose;
  std::string graspx_left_gripper;
  float txx, tyy, tzz;
  if (inputString2 == "box")
  {
      objecty = object; meshy_pose = mesh_pose;
      tzz = 0.07; tyy=0.01; txx=0.0;   
      graspx_left_gripper= "grasp1_left_gripper";
  }
  else if (inputString2 == "cube")
  {
      objecty = object4; meshy_pose = mesh4_pose;
      tzz = 0.06; tyy=0.03; txx=0.03;    
      graspx_left_gripper= "grasp_left_gripper";
  }
  else if (inputString2 == "ball")
  {
      objecty = object2; meshy_pose = mesh2_pose;
      tzz = 0.06; tyy=0.03; txx=0.03;    
      graspx_left_gripper= "grasp_left_gripper";
  }
  else if (inputString2 == "balloon")
  {
      objecty = object3; meshy_pose = mesh3_pose;
      tzz = 0.07; tyy=0.03; txx=0.03;   
      graspx_left_gripper= "grasp1_left_gripper";
  }
  else if (inputString2 == "chipbox")
  {
      objecty = object1; meshy_pose = mesh1_pose;
      tzz = 0.11; tyy=0.03; txx=0.03;   
      graspx_left_gripper= "grasp2_left_gripper";
  }
  else if(inputString2 == "teabox")
  {
      objecty = object5; meshy_pose = mesh5_pose;
      tzz = 0.11; tyy=0.035; txx=0.03;    
      graspx_left_gripper= "grasp2_left_gripper";
  }
  else {
  ROS_ERROR("Your choose is NOT correct!");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Press next and input the target of pick AGAIN!!", rvt::CYAN, rvt::XXLARGE); 
  visual_tools.trigger(); 
  visual_tools.prompt("Please choose the target within 'box' 'cube' 'ball' 'balloon' 'chipbox' and 'teabox' \n");
  // 从终端命令行获取抓取目标
  std::cin.clear(); 
  std::string inputString2;
  std::getline(std::cin, inputString2);
  }
  
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Get the taget, preparing to grasp \n Press next to pick ", rvt::BLUE, rvt::XXLARGE);
  visual_tools.trigger(); 
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // ***********************************************************************************************  
  // STEP 3: APPROACH CUP

  // Set up target pose for left arm.
  geometry_msgs::Pose approach2_pose = meshy_pose;    // Use target object pose (x,y,z) coordinates as reference.
  tf2::Quaternion orientation2;
  orientation2.setRPY(M_PI , 0.0 , 0.0);
  approach2_pose.orientation.x = orientation2.x();
  approach2_pose.orientation.y = orientation2.y();
  approach2_pose.orientation.z = orientation2.z();
  approach2_pose.orientation.w = orientation2.w();
  approach2_pose.position.z += tzz;        // Move 8 cm above the target object object.
  approach2_pose.position.x += txx;
  approach2_pose.position.y += tyy;

  arm_left_move_group.setStartState(*arm_left_move_group.getCurrentState());
  arm_left_move_group.setPoseTarget(approach2_pose);
  // Compute motion plan
  success = (arm_left_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Pick Tutorial: picking movements %s", success ? "SUCCESS" : "FAILED");

  // Visualizing plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approach2_pose, "Picking Position");
  visual_tools.publishText(text_pose, "Planning for object grasping \n Press next to perform motion on the real robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, arm_left_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Once user allow it, exceute the planned trajectory
  ROS_INFO("Performing motion on real robot...");
  arm_left_move_group.execute(my_plan);
  ros::Duration(1.5).sleep();

  //************************************************************************************************
  // STEP 3.1: The left gripper grasp.

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  left_gripper_move_group.setStartState(*left_gripper_move_group.getCurrentState());
  left_gripper_move_group.setNamedTarget("grasp_left_gripper");  // 
  success = (left_gripper_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.publishText(text_pose, "The left gripper grasp", rvt::WHITE, rvt::XXLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  left_gripper_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

//************************************************************************************************
// STEP 4: ATTACH THE OBJECT TO THE GRIPPER

  moveit::planning_interface::MoveGroupInterface left_gripper_mg("left_gripper");
  left_gripper_mg.attachObject(objecty.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object attached to the gripper \n Press next to place", rvt::PURPLE, rvt::XXLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Sleep to allow MoveGroup to recieve and process the attached collision object message
  // ros::Duration(1.0).sleep();

//************************************************************************************************
// STEP 5: PLACE THE OBJECT

  visual_tools.publishText(text_pose, "Placing Object inside the drum...\n (this might take a few seconds)", rvt::WHITE, rvt::XXLARGE);

  // Set drop position *******************************
  geometry_msgs::Pose drop2_pose = approach2_pose;
  tf2::Quaternion orient2;
  orient2.setRPY(M_PI , 0.0 , 0.0);
  drop2_pose.orientation = tf2::toMsg(orient2);
  // drop2_pose.orientation.x = orient2.x();
  // drop2_pose.orientation.y = orient2.y();
  // drop2_pose.orientation.z = orient2.z();
  // drop2_pose.orientation.w = orient2.w();
  drop2_pose.position.x = -0.1;    // 1.0            //[meters]
  drop2_pose.position.y =  0.6;    // 0.3            //[meters]
  drop2_pose.position.z =  0.76;   // 0.9            //[meters]
  arm_left_move_group.setStartState(*arm_left_move_group.getCurrentState());
  arm_left_move_group.setPoseTarget(drop2_pose, "arm_left_link_tcp");

  // This movement will involve the manipulation of the 15 joints at the same time, and will be a long sweep, that is why we give some seconds to the solver
  // to find a solution...
  arm_left_move_group.setPlanningTime(20.0);
  arm_left_move_group.setMaxAccelerationScalingFactor(0.5);
  arm_left_move_group.setMaxVelocityScalingFactor(0.3);
  arm_left_move_group.setNumPlanningAttempts(3);
  //Even do it will take time replanning improves results... we are not in a hurry.

  // Plan the motion with constraints.
  success = (arm_left_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // If no solution is found for dropping the object into the drum, roll back to home position and try again.
  if(!success){
    ROS_DEBUG("Planning from table to drum was not possible, going back to home position for replanning");
    // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
    csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
    csda10f_move_group.setNamedTarget("home_arms_folded");
    success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Planning to drop position not feasible \n Comming back to home position for replanning...", rvt::WHITE, rvt::XXLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");
    // Perform motion on real robot
    csda10f_move_group.execute(my_plan);
    ros::Duration(1.5).sleep();
    // Replan to target position
    ROS_DEBUG("Replanning to target position...");
    csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
    csda10f_move_group.setPoseTarget(drop2_pose, "arm_left_link_tcp");
    success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  if(!success){
    ROS_ERROR("No planning found to target position ... :c");
    return 1;
  }
  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approach2_pose, "start");
  visual_tools.publishAxisLabeled(drop2_pose, "goal");
  visual_tools.publishText(text_pose, "Perform motion of the cup mantaining orientation to avoid liquid spilling \n Press next to perform motion on real robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Once user allow it, exceute the planned trajectory
  ROS_INFO("Performing motion on real robot...");
  csda10f_move_group.execute(my_plan);
  ros::Duration(1.5).sleep();


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  left_gripper_mg.detachObject(objecty.id);

  // When done with the path constraint be sure to clear it.
  csda10f_move_group.clearPathConstraints();

  //************************************************************************************************
  // STEP 5.1: The left gripper open.

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  left_gripper_move_group.setStartState(*left_gripper_move_group.getCurrentState());
  left_gripper_move_group.setNamedTarget("open_left_gripper");
  success = (left_gripper_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // visual_tools.publishText(text_pose, "The left gripper open", rvt::WHITE, rvt::XXLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  visual_tools.deleteAllMarkers();
  left_gripper_move_group.execute(my_plan);
  ros::Duration(1.0).sleep();

  // *****************************************************************************************
  // STEP 6: PLAN TO PRE TEACH HOME POSITION

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
  csda10f_move_group.setNamedTarget("home_arms_folded");
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go back to home position \n Press next to perform motion", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  csda10f_move_group.execute(my_plan);
  visual_tools.deleteAllMarkers();
  ros::Duration(1.0).sleep();

  }

  // *****************************************************************************************
  // STEP 0: PLAN TO PRETEACH HOME POSITION

  // Use the previously defined home position for ease of motion, this position was defined on the moveit_config package
  csda10f_move_group.setStartState(*csda10f_move_group.getCurrentState());
  csda10f_move_group.setNamedTarget("home_arms_open");
  success = (csda10f_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Go back to origin position \n Press next to perform motion", rvt::WHITE, rvt::XXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, csda10f_joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press the 'next' button on the 'RvizVisualToolsGui' pannel");

  // Perform motion on real robot
  csda10f_move_group.execute(my_plan);
  visual_tools.deleteAllMarkers();
  ros::Duration(1.0).sleep();

}
  else
  {
    ROS_ERROR("Failed to get the objectdatas");
    return 1;
  }

  ros::shutdown();
  return 0;
}