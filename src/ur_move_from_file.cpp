#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>

#include <string>

#include <math.h>

#include <fstream>

using namespace std;

static const string PLANNING_GROUP= "manipulator";

static bool execute_flag = false;

static int stable_count = 0;

string file_trajectory_path = "/home/luk/catkin_ws/src/ur_move_scripts/trajectory/12.st";

//*************add CollisionObject table which is known being exist in real enviroment ***************//
void addObjectTable(std::vector<moveit_msgs::CollisionObject> &collision_objects, moveit_msgs::CollisionObject &table)
{
	table.id = "table";
	shape_msgs::SolidPrimitive table_shape;
	table_shape.type = table_shape.BOX;
	table_shape.dimensions.resize(3);
	table_shape.dimensions[0] = 1.5;
	table_shape.dimensions[1] = 1.5;
	table_shape.dimensions[2] = 0.2;


	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 0;
	table_pose.position.x = 0;
	table_pose.position.y = 0;
	table_pose.position.z = -0.1;

	table.primitives.push_back(table_shape);
	table.primitive_poses.push_back(table_pose);
	table.operation = table.ADD;

	collision_objects.push_back(table);

}

//*********initialize the manipulator to "home" position*********//

bool groupInit(moveit::planning_interface::MoveGroupInterface &group, moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{

	std::vector<double> init_pose;

	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), init_pose);
	init_pose[0] = 0.0;
	init_pose[1] = -1.57075;
	init_pose[2] = 1.57075;
	init_pose[3] = -1.57075;
	init_pose[4] = -1.57075;
	init_pose[5] = 0.0;

	group.setJointValueTarget(init_pose);
	bool success = group.plan(my_plan);

	if (success)
	{
		group.move();
		return true;
	}

	else 
	{
		ROS_ERROR("Unable to initialize robot pose! Please check the hardware!");
		return false;
	}

}

//****************add joint constriant*************//
void ocm_init(moveit::planning_interface::MoveGroupInterface &group)
{
	geometry_msgs::Pose pose_temp = group.getCurrentPose().pose;

    moveit_msgs::JointConstraint jcm;
    jcm.joint_name = "shoulder_lift_joint";
    jcm.position = -1.57;
    jcm.tolerance_above = 1.1;
    jcm.tolerance_below = 1.1;

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "ee_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = pose_temp.orientation.w;
	ocm.orientation.x = pose_temp.orientation.x;
	ocm.orientation.y = pose_temp.orientation.y;
	ocm.orientation.z = pose_temp.orientation.z;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	
	moveit_msgs::Constraints path_constraint;
	//path_constraint.orientation_constraints.push_back(ocm); //wrist pose constriant
	//path_constraint.joint_constraints.push_back(jcm);       //joint constriant

	group.setPathConstraints(path_constraint);

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_move_from_file");
	ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

	sleep(10.0);//wait to ensure Rviz has come up

	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	geometry_msgs::Pose target_pose1;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	moveit_msgs::CollisionObject table;
	table.header.frame_id = group.getPlanningFrame();
	addObjectTable(collision_objects, table);

	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);
	sleep(2.0);

    //read trajectory from file
    ifstream file_trajectory;
    file_trajectory.open(file_trajectory_path.c_str());

    string s;
    //while(file_trajectory >> s)
        //cout << s << endl;

    //initialize to home pose
	while(!groupInit(group, my_plan))
	{
		sleep(1.0);
		if(!node_handle.ok())
		{
			ros::shutdown();
			return 0;
		}
	}

	sleep(5.0);
	ocm_init(group);



	while (node_handle.ok())
	{
		target_pose1.position.x = 0.6 + 0.1;
		target_pose1.position.y = 0.6;
		target_pose1.position.z = 0.7 - 0.55;
		group.setPoseTarget(target_pose1);
		
		planning_scene_interface.addCollisionObjects(collision_objects);
		bool success = group.plan(my_plan);
		if (success)
			//group.move();



		ros::spinOnce();

/*****************
		if (1)
		{
	     ROS_INFO("Visualizing plan x:%f y:%f z:%f",  target_pose1.position.x,  target_pose1.position.z,  target_pose1.position.z );    
	     display_trajectory.trajectory_start = my_plan.start_state_;
	     display_trajectory.trajectory.push_back(my_plan.trajectory_);
	     display_publisher.publish(display_trajectory);
	     // Sleep to give Rviz time to visualize the plan. 
	     sleep(5.0);
		}
************/

	}	  

	ros::shutdown();
	return 0;
}


