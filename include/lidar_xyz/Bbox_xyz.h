/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2022                        
 * File: Bbox.h                          */

#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include<iostream>
#include "ros/ros.h"
#include<lidar_xyz/ClustersArray.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// Markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include<lidar_xyz/BoundingBox3D.h>

#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

class BoundingBox {
    private:
    ros::Publisher bbox_pub;
    ros::Publisher bbox_markers_pub;
    ros::Subscriber clusters_sub;
    tf::TransformListener tf_listener;
    // reference frame
    std::string reference_frame;
    std::string fixed_frame;
    bool oriented;
    double offset;
public:
    void clusters_callback(const lidar_xyz::ClustersArray::ConstPtr& clusters_msg);
    // function to find BBOX
    void getBBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, int j, visualization_msgs::Marker &marker, visualization_msgs::Marker &text_marker, lidar_xyz::BoundingBox3D &bbox);
    // Constructor
    BoundingBox(ros::NodeHandle *n);
    // Destructor
    ~BoundingBox();
    };
    
#endif