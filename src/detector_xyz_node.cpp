/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2021                        
 * File: detectorlidar_xyz_node.cpp                          */
 
#include <iostream>
#include "ros/ros.h"
#include "lidar_xyz/Detector_xyz.h"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "cloud_filter_node");
    
    ros::NodeHandle n1;
    Detector d(&n1);
    
    // Spin
    ros::spin ();
    
    return 0;
    }