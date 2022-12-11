#include "ros/ros.h"
#include "hand_mediapipe/FingerTipPose.h"
#include "hand_mediapipe/LandmarkPoint.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <iostream>


class PalmPoseServer
{
  private:
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hand_mediapipe::LandmarkPoint>("/my_gen3/landmark_server");
  ros::ServiceServer service = n.advertiseService("/my_gen3/palm_pose_server", &PalmPoseServer::get_palm_pose, this);

  public:
  PalmPoseServer(){}
  PalmPoseServer(ros::NodeHandle nh):n(nh){}

  bool get_palm_pose(hand_mediapipe::FingerTipPoseRequest &req, hand_mediapipe::FingerTipPoseResponse &res)
  {
    hand_mediapipe::LandmarkPoint srv;

    std::vector<geometry_msgs::Point> landmark_points;

    if (client.call(srv))
    {
      landmark_points = srv.response.LandmarkPoints;
    }

    Eigen::Vector3d nx, ny, nz, t;
    nz = {0,0,1};
    ny = {landmark_points[0].x - landmark_points[1].x, 
          landmark_points[0].y - landmark_points[1].y,
          landmark_points[0].z - landmark_points[0].z};

    nx = ny.cross(nz);

    double nx_mode = std::sqrt(nx[0]*nx[0] + nx[1]*nx[1] + nx[2]*nx[2]);
    double ny_mode = std::sqrt(ny[0]*ny[0] + ny[1]*ny[1] + ny[2]*ny[2]);
    double nz_mode = std::sqrt(nz[0]*nz[0] + nz[1]*nz[1] + nz[2]*nz[2]);

    Eigen::Vector4d nx_homo, ny_homo, nz_homo, t_homo;
    nx_homo = {nx[0]/nx_mode, nx[1]/nx_mode, nx[2]/nx_mode, 0};
    ny_homo = {ny[0]/ny_mode, ny[1]/ny_mode, ny[2]/ny_mode, 0};
    nz_homo = {nz[0]/nz_mode, nz[1]/nz_mode, nz[2]/nz_mode, 0};

    t_homo[0] = landmark_points[2].x - nz[0] * 0.2;
    t_homo[1] = landmark_points[2].y - nz[1] * 0.2;
    t_homo[2] = landmark_points[0].z - nz[2] * 0.2;
    t_homo[3] = 1.;
    // t_homo[0] = landmark_points[0].x;
    // t_homo[1] = landmark_points[0].y;
    // t_homo[2] = landmark_points[0].z;
    // t_homo[3] = 1;

    Eigen::Matrix4d tran;

    tran(0, 0) = nx_homo[0];  
    tran(0, 1) = ny_homo[0];
    tran(0, 2) = nz_homo[0];
    tran(0, 3) = t_homo[0];

    tran(1, 0) = nx_homo[1];
    tran(1, 1) = ny_homo[1];
    tran(1, 2) = nz_homo[1];
    tran(1, 3) = t_homo[1];
    
    tran(2, 0) = nx_homo[2];
    tran(2, 1) = ny_homo[2];
    tran(2, 2) = nz_homo[2];
    tran(2, 3) = t_homo[2];

    tran(3, 0) = nx_homo[3];
    tran(3, 1) = ny_homo[3];
    tran(3, 2) = nz_homo[3];
    tran(3, 3) = t_homo[3];
    
    Eigen::Affine3d affine;
    affine = tran;
    std::cout << affine.rotation() << std::endl;
    std::cout << tran << std::endl;
    tf::TransformListener listener;
    std::cout<<"1. 阻塞直到root 和 realsens  frame相通"<<std::endl;
    listener.waitForTransform("base_link","realsensed455",ros::Time(0),ros::Duration(4.0));
    tf::StampedTransform realsense2root;
    try{
     //2. 监听对应的tf,返回平移和旋转
    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
    listener.lookupTransform("/base_link", "/realsensed455",
        ros::Time(0), realsense2root);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    tf::StampedTransform ee2realsense;
    listener.waitForTransform("realsensed455","new_end_effector_link",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/realsensed455", "/new_end_effector_link",
            ros::Time(0), ee2realsense);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }



    tf::StampedTransform ee2caixuezhen;
    listener.waitForTransform("lancet","new_end_effector_link",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/lancet", "/new_end_effector_link",
            ros::Time(0), ee2caixuezhen);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }



    tf::Transform realsense2root_tf;
    tf::Transform ee2realsense_tf;
    tf::Transform ee2caixuezhen_tf;
    tf::Transform target2realsense_tf;
    tf::Transform realsense2ee_tf;

    tf::Transform target2root_tf;
    tf::Transform ee2root_tf;

    tf::Vector3 tf_t(affine.translation().x(), 
    affine.translation().y(), 
    affine.translation().z());


    tf::Matrix3x3 M33(
        affine.rotation().matrix()(0,0),
        affine.rotation().matrix()(0,1),
        affine.rotation().matrix()(0,2),
        affine.rotation().matrix()(1,0),
        affine.rotation().matrix()(1,1),
        affine.rotation().matrix()(1,2),
        affine.rotation().matrix()(2,0),
        affine.rotation().matrix()(2,1),
        affine.rotation().matrix()(2,2)
    );


    tf::Quaternion q2; 
    M33.getRotation(q2);
    target2realsense_tf.setOrigin(tf_t);
    target2realsense_tf.setRotation(q2);


    realsense2root_tf.setOrigin(realsense2root.getOrigin());
    realsense2root_tf.setRotation(realsense2root.getRotation());

    ee2realsense_tf.setOrigin(ee2realsense.getOrigin());
    ee2realsense_tf.setRotation(ee2realsense.getRotation());

    ee2caixuezhen_tf.setOrigin(ee2caixuezhen.getOrigin());
    ee2caixuezhen_tf.setRotation(ee2caixuezhen.getRotation());

    target2root_tf.mult(realsense2root_tf,target2realsense_tf);
    ee2root_tf.mult(target2root_tf, ee2realsense_tf);


    res.pose.position.x = ee2root_tf.getOrigin().x();
    res.pose.position.y = ee2root_tf.getOrigin().y();
    res.pose.position.z = ee2root_tf.getOrigin().z();
    res.pose.orientation.x = ee2root_tf.getRotation().getX();
    res.pose.orientation.y = ee2root_tf.getRotation().getY();
    res.pose.orientation.z = ee2root_tf.getRotation().getZ();
    res.pose.orientation.w = ee2root_tf.getRotation().getW();
    std::cout << "======================================" << std::endl;
    std::cout << res.pose.position.x << std::endl;
    std::cout << res.pose.position.y << std::endl;
    std::cout << res.pose.position.z << std::endl;
    std::cout << res.pose.orientation.x << std::endl;
    std::cout << res.pose.orientation.y << std::endl;
    std::cout << res.pose.orientation.z << std::endl;
    std::cout << res.pose.orientation.w << std::endl;

  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fingertip_pose_server_node");
  ros::NodeHandle n;
  PalmPoseServer server = PalmPoseServer(n);

  ros::spin();
  return 0;
}