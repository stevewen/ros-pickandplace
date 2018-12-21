#include <ros/ros.h>
#include "robot_camera/object_struct_sc.h"
#include "std_msgs/String.h"
#include <stdio.h>
// #include <stdlib.h>
// #include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Quaterniond pose(float Pa, float Pb, float Pc, float Rx, float Ry, float Rz)   
{
     Eigen::Vector3d  vx0, vx, vy, vz;      
     vx0 << 1,  0,  0; 
     vz  << Pa, Pb, Pc; 
     Eigen::Vector3d vznorm = vz.normalized();  
     vy = vznorm.cross(vx0);
     Eigen::Vector3d vynorm = vy.normalized();  
     vx = vynorm.cross(vznorm);
     Eigen::Vector3d vxnorm = vx.normalized();  

     Eigen::Matrix3d  Mat;  
     Mat << vxnorm[0], vynorm[0], vznorm[0],                                            
            vxnorm[1], vynorm[1], vznorm[1],
            vxnorm[2], vynorm[2], vznorm[2];

     Eigen::Quaterniond q;   
     q = Mat;
     std::cout<< "Quaternion" << std::endl << q.coeffs() << std::endl;

     return q;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robot_camera::object_struct_sc>("object_struct_sc");

  robot_camera::object_struct_sc srv;
  srv.request.ifstart = "ON";

  if(client.call(srv))
  {
      int num = srv.response.Class.size();    
      float res_x[num],res_y[num],res_z[num]; 
      float ori_x[num],ori_y[num],ori_z[num]; 
      std::string res_name[num];   
      
      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.Class[i]<<" ";
        res_name[i] = srv.response.Class[i];
      }
      std::cout<<std::endl;

      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.center_x[i]<<" ";
        res_x[i] = srv.response.center_x[i];
      }
      std::cout<<std::endl;
      
      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.center_y[i]<<" ";
        res_y[i] = srv.response.center_y[i];
      }
      std::cout<<std::endl;

      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.center_z[i]<<" ";
        res_z[i] = srv.response.center_z[i];
      }
      std::cout<<std::endl;

      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.front_x[i]<<" ";
        ori_x[i] = srv.response.front_x[i];
      }
      std::cout<<std::endl;
      
      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.front_y[i]<<" ";
        ori_y[i] = srv.response.front_y[i];
      }
      std::cout<<std::endl;

      for (int i = 0; i < num; i++)
      {
        std::cout<<srv.response.front_z[i]<<" ";
        ori_z[i] = srv.response.front_z[i];
      }
      std::cout<<std::endl;

      float Pl_a, Pl_b, Pl_c, Pl_d;     
      Pl_a = srv.response.plane_a;
      Pl_b = srv.response.plane_b;
      Pl_c = srv.response.plane_c;
      Pl_d = srv.response.plane_d;
      std::cout<<Pl_a<<" "<<Pl_b<<" "<<Pl_c<<" "<<Pl_d<<std::endl;

      Eigen::Quaterniond ori  = pose(Pl_a, Pl_b, Pl_c, ori_x[0], ori_y[0], ori_z[0]);
      Eigen::Quaterniond ori1 = pose(Pl_a, Pl_b, Pl_c, ori_x[1], ori_y[1], ori_z[1]);
      Eigen::Quaterniond ori2 = pose(Pl_a, Pl_b, Pl_c, ori_x[2], ori_y[2], ori_z[2]);
      Eigen::Quaterniond ori3 = pose(Pl_a, Pl_b, Pl_c, ori_x[3], ori_y[3], ori_z[3]);
      Eigen::Quaterniond ori4 = pose(Pl_a, Pl_b, Pl_c, ori_x[4], ori_y[4], ori_z[4]);
      Eigen::Quaterniond ori5 = pose(Pl_a, Pl_b, Pl_c, ori_x[5], ori_y[5], ori_z[5]);
      
  }

  else std::cout<<"It is failed~"<<std::endl;

  return 0; 

}
