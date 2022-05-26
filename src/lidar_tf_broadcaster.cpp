#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_tf_broadcaster");

    ros::NodeHandle node;
    ros::Rate rate(150);
    
    while (node.ok()){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 1.25) );
        tf::Quaternion q;
        q.setRPY(0, 0.279253, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "velodyne"));
        rate.sleep();
        }
    
    ros::spin();
    return 0;
};