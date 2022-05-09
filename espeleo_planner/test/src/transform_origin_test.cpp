#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_espeleo_tf_listener");

    ros::NodeHandle node;

    tf::TransformListener listener;

    ros::Rate rate(2.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("os1_init", "base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        ROS_INFO_STREAM(transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() );
        rate.sleep();
    }
    return 0;
};