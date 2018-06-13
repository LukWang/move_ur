#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.141592657

using std::vector;
using std::cout;
using std::endl;

bool evaluateTransform(tf::StampedTransform& transform_candidate, tf::Transform& transform_previous)
{
    Eigen::Matrix<double, 3, 1> trans_diff;
    trans_diff << transform_candidate.getOrigin().x() - transform_previous.getOrigin().x(),
                  transform_candidate.getOrigin().y() - transform_previous.getOrigin().y(),
                  transform_candidate.getOrigin().z() - transform_previous.getOrigin().z();
    if (trans_diff.norm() > 0.05)
    {
        ROS_INFO("Target position Changed!");
        return true;
    }
    tf::Quaternion q_tf = transform_candidate.getRotation();
    Eigen::Quaterniond q_cand(q_tf[3], q_tf[0], q_tf[1], q_tf[2]);
    q_tf = transform_previous.getRotation();
    Eigen::Quaterniond q_prev(q_tf[3], q_tf[0], q_tf[1], q_tf[2]);
    Eigen::Matrix3d r_cand = q_cand.toRotationMatrix();
    Eigen::Matrix3d r_prev = q_prev.toRotationMatrix();
    Eigen::Matrix3d r_res;
    r_res = q_cand.inverse() * q_prev;
    Eigen::Quaterniond q_res;
    q_res = r_res;

    double res = q_res.x() * q_res.x() + 
                 q_res.y() * q_res.y() +
                 q_res.z() * q_res.z();
    if (res > 0.1)
    {
        ROS_INFO("Target Rotation Changed!");
        return true;
    }
    else
        return false;

}

void* goal_generate_thread(void* arg)
{
    static tf::TransformBroadcaster br;
    tf::Transform fake_goal;
    fake_goal.setOrigin(tf::Vector3(0.0, 0.0, 0.3));
    tf::Quaternion q;
    q.setRPY(PI, 0.0, 0.0);
    fake_goal.setRotation(q);

    ros::Rate rate(30);
    while(1)
    {    
        br.sendTransform(tf::StampedTransform(fake_goal, ros::Time::now(), "marker_10", "fake_goal"));
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_cmd_publisher");

    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    tf::TransformListener listener;
    static tf::TransformBroadcaster br;


    //tf::StampedTransform base_tool_transform;
    tf::StampedTransform base_goal_transform;

    ros::Publisher cmd_publisher = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);

    tf::Vector3 t_goal_previous;
    tf::Quaternion q_goal_previous;
    
    tf::Vector3 t_goal;
    tf::Quaternion q_goal;


    tf::Quaternion q_tool;
    tf::Quaternion q_goal_candidate;
    
    vector<double> cmd_vector(6);

    tf::Transform transform_previous;
    transform_previous.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform_previous.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    pthread_t id1;
    pthread_create(&id1, NULL, goal_generate_thread, NULL);

    std::string stop_cmd = "stopj(0.1)";
    
    while(nh.ok())
    {
        
        try{
            listener.lookupTransform("/base", "/fake_goal", ros::Time(0), base_goal_transform); 
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR(ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        bool doNewMove =  evaluateTransform(base_goal_transform, transform_previous);
        if(doNewMove)
        {
            std_msgs::String cmd_msg;
            cmd_msg.data = stop_cmd.c_str();
            ROS_INFO("ur script send: %s", stop_cmd.c_str());
        
            cmd_publisher.publish(cmd_msg);
            
            
            t_goal = base_goal_transform.getOrigin();
            q_goal = base_goal_transform.getRotation();
            
            transform_previous.setOrigin(t_goal);
            transform_previous.setRotation(q_goal);

            double goal_angle = q_goal.getAngle();
            tf::Vector3 goal_axis = q_goal.getAxis(); 

            if(goal_angle > PI)
            {
                goal_angle = -(2 * PI - goal_angle);
            }
            
            goal_axis *= goal_angle;
        
                
            cmd_vector.clear();
            cmd_vector.push_back(t_goal.x());
            cmd_vector.push_back(t_goal.y());
            cmd_vector.push_back(t_goal.z());
            cmd_vector.push_back(goal_axis.getX());
            cmd_vector.push_back(goal_axis.getY());
            cmd_vector.push_back(goal_axis.getZ());
        
            char cmd[100];
        
            //sprintf(cmd, "speedl([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 0.2, 0.1)\n", cmd_vector[0],cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
            sprintf(cmd, "movel(p[%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 0.2, 0.1, 0.0, 0.0)\n", cmd_vector[0],cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
            ROS_INFO("ur script send: %s", cmd);

            //std_msgs::String cmd_msg;
            cmd_msg.data = cmd;
        
            cmd_publisher.publish(cmd_msg);
            
        } 

        rate.sleep();

    }

    return 0;
}
