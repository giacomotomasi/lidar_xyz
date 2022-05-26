/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2021                        
 * File: Bboxlidar_xyz.cpp                          */

#include<iostream>

#include<ros/ros.h>
#include<lidar_xyz/ClustersArray.h>
#include<sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
// Markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// tf
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "lidar_xyz/Bbox_xyz.h"


void BoundingBox_moi::clusters_callback(const lidar_xyz::ClustersArray::ConstPtr& clusters_msg){
    visualization_msgs::MarkerArray::Ptr bbox_markers (new visualization_msgs::MarkerArray);
    //std::cout << (*clusters_msg).clusters.size() << std::endl;
    for (int i {0};i<(*clusters_msg).clusters.size();i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // convert cloud to pcl::PointXYZRGB
        pcl::fromROSMsg((*clusters_msg).clusters.at(i), *cloud);
        visualization_msgs::Marker marker, text_marker;
        tie(marker, text_marker) = getBBox(cloud, i);
        (*bbox_markers).markers.push_back(marker);
        (*bbox_markers).markers.push_back(text_marker);
        }
    rviz_visual_tools::RvizVisualTools *visual_toolPtr = new rviz_visual_tools::RvizVisualTools("base_link", "bbox_marker");
    visual_toolPtr->deleteAllMarkers();
    delete visual_toolPtr;
    bbox_pub.publish(bbox_markers);
    
}

// function to find BBOX
std:: tuple<visualization_msgs::Marker, visualization_msgs::Marker> BoundingBox_moi::getBBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, int j){

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cluster);
    feature_extractor.compute();
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    // create marker correspoinding to the bbox
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame;
    marker.ns = "Obstacle";
    marker.header.stamp = ros::Time::now();
    marker.id = j;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);
    marker.pose.position.x = position_OBB.x;
    marker.pose.position.y = position_OBB.y;
    marker.pose.position.z = position_OBB.z;
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = max_point_OBB.x - min_point_OBB.x + 0.02;
    marker.scale.y = max_point_OBB.y - min_point_OBB.y + 0.02;
    marker.scale.z = max_point_OBB.z - min_point_OBB.z + 0.02;
    marker.color.a = 0.15;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0;    
    // create TEXT marker
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = reference_frame;
    std::stringstream obs;
    obs << "Obstacle " << j;
    text_marker.text = obs.str();
    text_marker.ns = "Obstacle";
    text_marker.header.stamp = ros::Time::now();
    text_marker.id = j+100;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.lifetime = ros::Duration(0);
    text_marker.pose.position.x = position_OBB.x;
    text_marker.pose.position.y = position_OBB.y - ((max_point_OBB.x - min_point_OBB.x)/2 + 0.04);
    text_marker.pose.position.z = position_OBB.z;
    text_marker.pose.orientation.x = quat.x();
    text_marker.pose.orientation.y = quat.y();
    text_marker.pose.orientation.z = quat.z();
    text_marker.pose.orientation.w = quat.w();
    text_marker.scale.x = 0.04;
    text_marker.scale.y = 0.04;
    text_marker.scale.z = 0.04;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0;
    return {marker, text_marker};
    }
    
// Constructor
BoundingBox_moi::BoundingBox_moi(ros::NodeHandle *n){
    std::cout << "\033[1;32m BoundingBox constructor called.\033[0m" << std::endl;
    // get ros parameters
    n->param<std::string>("/reference_frame/frame_id",reference_frame,"velodyne");
    bbox_pub = n->advertise<visualization_msgs::MarkerArray>("bbox_marker", 1);
    clusters_sub = n->subscribe("pcl_clusters", 1, &BoundingBox_moi::clusters_callback, this);
    }
    
// Destructor
BoundingBox_moi::~BoundingBox_moi(){
    std::cout << "\033[1;32m BoundingBox destructor called.\033[0m" << std::endl; 
    };
    
    
    
void BoundingBox_pca::clusters_callback(const lidar_xyz::ClustersArray::ConstPtr& clusters_msg){
    visualization_msgs::MarkerArray::Ptr bbox_markers (new visualization_msgs::MarkerArray);
    //std::cout << (*clusters_msg).clusters.size() << std::endl;
    for (int i {0};i<(*clusters_msg).clusters.size();i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // convert cloud to pcl::PointXYZRGB
        pcl::fromROSMsg((*clusters_msg).clusters.at(i), *cloud);
        visualization_msgs::Marker marker, text_marker;
        tie(marker, text_marker) = getBBox(cloud, i);
        (*bbox_markers).markers.push_back(marker);
        (*bbox_markers).markers.push_back(text_marker);
        }
    bbox_pub.publish(bbox_markers);
}

// function to find BBOX
std:: tuple<visualization_msgs::Marker, visualization_msgs::Marker> BoundingBox_pca::getBBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, int j){

    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *projected_cloud);
    Eigen::Matrix3f eigen_vector_pca = pca.getEigenVectors();
    Eigen::Vector3f eigen_values = pca.getEigenValues();
    Eigen::Matrix3d eigen_vector_pca_double = eigen_vector_pca.cast<double>();
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*projected_cloud, min_point, max_point);
    Eigen::Vector4f cluster_centroid;
    pcl::compute3DCentroid(*cluster, cluster_centroid);
    tf::Quaternion quat;
    tf::Matrix3x3 tf_rotation;
    tf::matrixEigenToTF(eigen_vector_pca_double, tf_rotation);
    tf_rotation.getRotation(quat);
    // create marker correspoinding to the bbox
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame;
    marker.ns = "Obstacle";
    marker.header.stamp = ros::Time::now();
    marker.id = j;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);
    marker.pose.position.x = cluster_centroid(0);
    marker.pose.position.y = cluster_centroid(1);
    marker.pose.position.z = cluster_centroid(2);
    marker.pose.orientation.x = quat.getAxis().getX();
    marker.pose.orientation.y = quat.getAxis().getY();
    marker.pose.orientation.z = quat.getAxis().getZ();
    marker.pose.orientation.w = quat.getW();
    marker.scale.x = max_point.x - min_point.x;
    marker.scale.y = max_point.y - min_point.y;
    marker.scale.z = max_point.z - min_point.z;
    marker.color.a = 0.15;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0;
    // create TEXT marker
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = reference_frame;
    std::stringstream obs;
    obs << "Obstacle " << j;
    text_marker.text = obs.str();
    text_marker.ns = "Obstacle";
    text_marker.header.stamp = ros::Time::now();
    text_marker.id = j+100;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.lifetime = ros::Duration(0);
    text_marker.pose.position.x = cluster_centroid(0);
    text_marker.pose.position.y = cluster_centroid(1) - ((max_point.x - min_point.x)/2 + 0.04);;
    text_marker.pose.position.z = cluster_centroid(2);
    text_marker.pose.orientation.x = quat.getAxis().getX();
    text_marker.pose.orientation.y = quat.getAxis().getY();
    text_marker.pose.orientation.z = quat.getAxis().getZ();
    text_marker.pose.orientation.w = quat.getW();
    text_marker.scale.x = max_point.x - min_point.x;
    text_marker.scale.y = max_point.y - min_point.y;
    text_marker.scale.z = max_point.z - min_point.z;
    text_marker.scale.x = 0.04;
    text_marker.scale.y = 0.04;
    text_marker.scale.z = 0.04;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0;
    return {marker, text_marker};
    }
    
    // Constructor
    BoundingBox_pca::BoundingBox_pca(ros::NodeHandle *n){
    std::cout << "\033[1;32m BoundingBox constructor called.\033[0m" << std::endl;
    // get ros parameters
    n->param<std::string>("/reference_frame/frame_id",reference_frame,"velodyne");
    bbox_pub = n->advertise<visualization_msgs::MarkerArray>("bbox_marker", 1);
    clusters_sub = n->subscribe("pcl_clusters", 1, &BoundingBox_pca::clusters_callback, this);
    }
    
    // Destructor
    BoundingBox_pca::~BoundingBox_pca(){
    std::cout << "\033[1;32m BoundingBox destructor called.\033[0m" << std::endl; 
    }
    