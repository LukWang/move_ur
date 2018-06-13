#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#define JOINT_NUM 6

std::vector<std::string> joint_names(JOINT_NUM);

std::vector<double> joint_angles(JOINT_NUM);

void jointStateCallback(const sensor_msgs::JointState& joint_state)
{
    joint_angles.clear();
    std::vector<std::string> joint_names_recv = joint_state.name;
    for(auto it = joint_names.begin(); it !=joint_names.end(); ++it)
    {
        for(auto it_recv = joint_names_recv.begin(); it_recv != joint_names_recv.end(); ++it_recv)
        {
            if (*it_recv == *it)
            {
                int idx = it_recv - joint_names_recv.begin();
                int i = it - joint_names_recv.begin();
                joint_angles.push_back(joint_state.position[idx]);
                break;
            }
        }
    }

}

int main(int argc, char** argv)
{
    joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");


    ros::init(argc, argv, "ur_move_cmd");

    ros::NodeHandle nh;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);


    KDL::Tree UR_tree;

    KDL::Chain ur_chain;
    
    KDL::JntArray q;

    KDL::Jacobian J;

    q.resize(JOINT_NUM);

    J.resize(JOINT_NUM);

    kdl_parser::treeFromFile("/home/luk/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf", UR_tree);
    //kdl_parser::treeFromFile("/home/luk/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf.xacro", UR_tree);

    bool exit_value;
    exit_value = UR_tree.getChain("base", "tool0", ur_chain);
    if(exit_value)
        printf("UR Chain get");

    KDL::ChainJntToJacSolver jnt_to_jac_solver(ur_chain);



    ros::spinOnce();

    ros::Rate rate(100);
    while(ros::ok())
    {
        if(!joint_angles.empty())
        {
            for(int i = 0; i < JOINT_NUM; ++i)
            {
                q(i) = joint_angles[i];
            }
            ROS_INFO("joint info:[%.4f %.4f %.4f %.4f %.4f %.4f]\n", q(0), q(1), q(2), q(3), q(4), q(5));
            jnt_to_jac_solver.JntToJac(q, J);
        }


        ros::spinOnce();
    }

    
    return 0;
}
