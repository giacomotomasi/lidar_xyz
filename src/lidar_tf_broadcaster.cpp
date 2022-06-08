/*
 * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: April 2022                        
 * File: lidar_tf_broadcaster.cpp                          */
 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_tf_broadcaster");

    ros::NodeHandle node;
    ros::Rate rate(150);
    double x {}, y {}, z {}, roll {}, pitch {}, yaw {};
    
    node.param("/transform/position/x",x,0.0);
    node.param("/transform/position/y",y,0.0);
    node.param("/transform/position/z",z,0.0);
    node.param("/transform/orientation/roll",roll,0.0);
    node.param("/transform/orientation/pitch",pitch,0.0);
    node.param("/transform/orientation/yaw",yaw,0.0);
    
    while (node.ok()){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(x, y, z) );
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "velodyne"));
        rate.sleep();
        }
    
    ros::spin();
    return 0;
};