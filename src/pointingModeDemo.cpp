#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <trac_ik/trac_ik.hpp>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

#include <tf_conversions/tf_kdl.h>

#include <pthread.h>

#include <cmath>

#include <std_msgs/String.h>

#include <ros_myo/MyoGesture.h>

#include <robotiq_s_model_control/SModel_robot_output.h>

#define JOINT_NUM 6

#define PUB_RATE 20

#define TIME_STEP 1.0/PUB_RATE

#define GO_DOWN 1

#define CLOSE_CLAW 2

#define OPEN_CLAW 3

#define STOP 0

using std::string;

std::vector<std::string> joint_names(JOINT_NUM);

std::vector<double> joint_angles(JOINT_NUM);

std::vector<double> joint_speed(JOINT_NUM);

int claw_cmd = 0;

bool JointState_available;


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
                joint_speed.push_back(joint_state.velocity[idx]);
                break;
            }
        }
    }
    JointState_available = true;
}


void* goal_generate_thread(void *arg)
{
    ros::NodeHandle* nh = (ros::NodeHandle*) arg;
    tf::TransformBroadcaster br;
    tf::Transform goal;
    goal.setOrigin(tf::Vector3(0.0,0.0,0.1));
    tf::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    goal.setRotation(q);
    std::string goal_frame;
    ros::Rate rate(30);
    while(ros::ok()){
        nh->param<std::string>("goal_frame", goal_frame, "Marker_10");

        br.sendTransform(tf::StampedTransform(goal, ros::Time::now(), goal_frame.c_str(), "desired_goal"));
        rate.sleep();
    }
}


void MyoGestureCallback(ros_myo::MyoGesture gest)
{
    if(gest.gesture == 3)
        claw_cmd = GO_DOWN;
    else if(gest.gesture == 1)
        claw_cmd = STOP;
    else if(gest.gesture == 2)
        claw_cmd = CLOSE_CLAW;
    else if(gest.gesture == 5)
        claw_cmd = OPEN_CLAW;
}


void* goal_rectify_thread(void *arg)
{
    tf::TransformBroadcaster br;
    tf::Transform rect_goal;

    tf::TransformListener tf_listener;
    tf::StampedTransform desired_goal;

    double work_space_radius = 0.75;
    double base_radius = 0.2;

    ros::Rate rate(60);
    while(ros::ok()){
        try{
            tf_listener.lookupTransform("base", "desired_goal", ros::Time(0), desired_goal);

            tf::Vector3 goal_position = desired_goal.getOrigin();
            tf::Vector3 rectified_position = goal_position;

            rect_goal.setRotation(desired_goal.getRotation());
            rect_goal.setOrigin(desired_goal.getOrigin());
            if (goal_position.z() < 0.2){
                rectified_position.setZ(0.2);
                rect_goal.setOrigin(rectified_position);
                ROS_WARN("goal under the working surface, setting rectified goal\n");
            }

            double desired_distance = rectified_position.length();

            if (desired_distance > work_space_radius)
            {
                double scale = work_space_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal outoff robot work space! setting rectified goal\n");
            }

            desired_distance = rectified_position.length();
            if (desired_distance < base_radius){
                double scale = base_radius / desired_distance;
                rectified_position = tf::Vector3(rectified_position.x() * scale,
                                               rectified_position.y() * scale,
                                               rectified_position.z() * scale);
                rect_goal.setOrigin(rectified_position);

                ROS_WARN("goal too close to base! setting rectified goal\n");
            }
            br.sendTransform(tf::StampedTransform(rect_goal, ros::Time::now(), "base", "rect_goal"));
        }
        catch(tf::TransformException ex){
        }
        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    bool publish_goal = true;
    if(argc > 1)
    {
        std::string arg;
        arg = argv[1];
        if(arg == "false"){
            publish_goal = false;
            ROS_INFO("Publishing goal disabled");
        }
    }


    joint_names.push_back("shoulder_pan_joint");
    joint_names.push_back("shoulder_lift_joint");
    joint_names.push_back("elbow_joint");
    joint_names.push_back("wrist_1_joint");
    joint_names.push_back("wrist_2_joint");
    joint_names.push_back("wrist_3_joint");

    /***PID parameters***/


    double Kp = 0.3;

    double Td = 2.5;

    double max_speed = 0.8;


    std_msgs::String cmd_msg;

    ros::init(argc, argv, "ur_move_cmd");

    ros::NodeHandle nh;

    tf::TransformListener tf_listener;

    if(publish_goal){
        pthread_t goal_thread;
        pthread_create(&goal_thread, NULL, goal_generate_thread, (void*)&nh);
    }

    pthread_t rectify_thread;
    pthread_create(&rectify_thread, NULL, goal_rectify_thread, NULL);

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);

    ros::Subscriber gesture_sub = nh.subscribe("/myo_raw/gest", 1, MyoGestureCallback);

    ros::Publisher ur_cmd_publisher = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);

    ros::Publisher claw_cmd_publisher = nh.advertise<robotiq_s_model_control::SModel_robot_output>("/SModelRobotOutput", 1);

    robotiq_s_model_control::SModel_robot_output robotiq_cmd;
    robotiq_cmd.rACT = 1;
    robotiq_cmd.rMOD = 0;
    robotiq_cmd.rGTO = 1;
    robotiq_cmd.rATR = 0;
    robotiq_cmd.rGLV = 0;
    robotiq_cmd.rICF = 0;
    robotiq_cmd.rICS = 0;
    robotiq_cmd.rPRA = 255;
    robotiq_cmd.rSPA = 255;
    robotiq_cmd.rFRA = 25;
    robotiq_cmd.rPRB = 0;
    robotiq_cmd.rSPB = 0;
    robotiq_cmd.rFRB = 0;
    robotiq_cmd.rPRC = 0;
    robotiq_cmd.rFRC = 0;
    robotiq_cmd.rPRS = 0;
    robotiq_cmd.rSPS = 0;
    robotiq_cmd.rFRS = 0;

    string urdf_param = "/robot_description";

    string base = "base";
    string tip = "tool0";
    //kdl_parser::treeFromFile("/home/luk/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf", UR_tree);
    //kdl_parser::treeFromFile("/home/luk/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf.xacro", UR_tree);

    TRAC_IK::TRAC_IK ik_solver(base, tip, urdf_param, 0.005, 1e-5, TRAC_IK::Distance);

    KDL::JntArray q(6);

    KDL::JntArray q_desired(6);

    KDL::Frame desired_pose;

    tf::StampedTransform transform;

    bool tf_available = false;


    //move to initial pose

    ros::spinOnce();

    ros::Rate rate(20);



    std::cout << "controlling using time step at: " << TIME_STEP << std::endl;




    while(ros::ok())
    {

        if(claw_cmd == GO_DOWN){

            std::string cmd = "speedl([0.0, 0.0, -0.05, 0.0, 0.0, 0.0], 0.2, 0.1)";
            std::string stop_cmd = "stopj(1.0)";
            cmd_msg.data = cmd.c_str();
            ros::Rate rate(100);
            while(ros::ok && claw_cmd != STOP)
            {
                ur_cmd_publisher.publish(cmd_msg);
                ros::spinOnce();
                rate.sleep();
            }
            cmd_msg.data = stop_cmd.c_str();
            ur_cmd_publisher.publish(cmd_msg);
            while(claw_cmd != CLOSE_CLAW && ros::ok()){
                ros::spinOnce();
                rate.sleep();
            }
            robotiq_cmd.rPRA = 255;
            robotiq_cmd.rSPA = 255;
            claw_cmd_publisher.publish(robotiq_cmd);

            sleep(3.0);
        }

        if(claw_cmd == OPEN_CLAW){

            robotiq_cmd.rPRA = 0;
            robotiq_cmd.rSPA = 255;



            claw_cmd_publisher.publish(robotiq_cmd);

            sleep(3.0);
        }


        try{
            tf_listener.lookupTransform("base", "rect_goal", ros::Time(0), transform);
            transformTFToKDL(transform, desired_pose);
            tf_available = true;
        }
        catch(tf::TransformException ex){
            continue;
            tf_available = false;
        }
        if(JointState_available && tf_available)
        {
            for(int i = 0; i < JOINT_NUM; ++i)
            {
                q(i) = joint_angles[i];
            }
            if(ik_solver.CartToJnt(q, desired_pose, q_desired))
            {
                std::vector<double> cmd_vector;
                for(int i = 0; i < JOINT_NUM; ++i)
                {
                    double delta = q_desired(i) - q(i);
                    double speed = Kp * delta - Td * joint_speed[i];
                    if (speed > max_speed)
                        speed = max_speed;
                    if (speed < -max_speed)
                        speed = -max_speed;
                    cmd_vector.push_back(speed);
                }
                char cmd[100];
                sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 1.0, 0.05)", cmd_vector[0], cmd_vector[1], cmd_vector[2], cmd_vector[3], cmd_vector[4], cmd_vector[5]);
                ROS_INFO("ur script send: %s", cmd);

                cmd_msg.data = cmd;
                ur_cmd_publisher.publish(cmd_msg);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
