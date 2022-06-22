/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2022                        
 * File: Detector.h                          */

#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class Detector {
private:
    ros::Publisher cloud_pub;
    ros::Publisher clusters_pub;
    ros::Subscriber cloud_sub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    tf::TransformListener tf_listener;
    // pointcloud camera topic
    std::string pointcloud_topic;
    // reference frame
    std::string reference_frame;
    // voxel grid parameters
    double size_x {};
    double size_y {};
    double size_z {};
    bool voxel_grid_enabled;
    // pass through parameters
    double x_min {};
    double x_max {};
    bool x_pass_through_enabled;
    double y_min {};
    double y_max {};
    bool y_pass_through_enabled;
    double z_min {};
    double z_max {};
    bool z_pass_through_enabled;
    // segmentation parameters
    double distance_threshold{};
    bool segmentation_enabled;
    // outlier removal parameters
    int meanK {};
    double standard_dev_mult {};
    bool outlier_removal_enabled;
    // cluster extraction parameters
    double cluster_tolerance {};
    int min_cluster_size {};
    int max_cluster_size {};
    bool cluster_extraction_enabled;
    // transform
    double x {};
    double y {};
    double z {};
    double roll {};
    double pitch {};
    double yaw {};
    double robot_x {};
    double robot_y {};
    double robot_qx {};
    double robot_qy {};
    double robot_qz {};
    double robot_qw {};
public:
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void voxel_grid();
    void pass_through();
    void segmentation();
    void extract_indices(pcl::PointIndices::Ptr indices, bool mode);
    void outlier_removal();
    void cluster_extraction();
    void publish();
    // Constructor
    Detector(ros::NodeHandle *n1);
    // Destructor
    ~Detector();
    };

#endif