#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>


void poseCallback(const turtlesim::PoseConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 1.25) );
    tf::Quaternion q;
    q.setRPY(0, 0.279253, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;
};