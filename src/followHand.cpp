#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "follow_frame_node");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    tf::StampedTransform tool_transform;
    tf::StampedTransform goal_transform;
    std::string goal_frame;
    while(ros::ok()){
        if(nh.getParam("goal_frame", goal_frame)){
            ROS_INFO("Got goal frame : %s", goal_frame.c_str());
            try{
                listener.lookupTransform("base", "tool0", ros::Time(0), tool_transform);
                listener.lookupTransform("base", goal_frame.c_str(), ros::Time(0), goal_transform);
                
                break;
            }
            catch(tf::TransformException ex){
                sleep(1);
                continue;
            }
        
        }
    }
    tf::Vector3 offset(tool_transform.getOrigin().x() - goal_transform.getOrigin().x(),
                       tool_transform.getOrigin().y() - goal_transform.getOrigin().y(),
                       tool_transform.getOrigin().z() - goal_transform.getOrigin().z());
    
    static tf::TransformBroadcaster br;
    tf::Quaternion q;
    tf::Transform desired_goal_transform;
    ros::Rate rate(30);
    while(ros::ok)
    {
        try{
            listener.lookupTransform("base", goal_frame.c_str(), ros::Time(0), goal_transform);
            desired_goal_transform.setOrigin(tf::Vector3(goal_transform.getOrigin().x() + offset.x(),
                                                         goal_transform.getOrigin().y() + offset.y(),
                                                         goal_transform.getOrigin().z() + offset.z()));
            q = goal_transform.getRotation();
            desired_goal_transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(desired_goal_transform, ros::Time::now(), "base", "desired_goal"));
        }
        catch(tf::TransformException ex){
        
        }
        rate.sleep();
    }

    return 0;
}
