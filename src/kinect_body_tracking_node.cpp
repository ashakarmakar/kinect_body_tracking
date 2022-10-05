#include <MultiKinectWrapper.h>
#include <MKConfig.h>
#include "AsyncMKPRSplitter.h"
#include "MKPPersonTracker.h"
#include "MKPComputeGoal.h"
#include "DepthViewer.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    MKConfig config(1);
    MultiKinectWrapper wrapper(config);
    AsyncMKPRSplitter split;    
    wrapper.setRecipient(&split);

    /* TODO: node handle, ros init */
    
    /* 1. PersonTracker: will collect joint data */ 
    MKPPersonTracker person_tracker(wrapper);
    split.addRecipient(&person_tracker);

    /* 2. Goal Computer: will create a movebase goal based on the joint data */
    MKPComputeGoal compute_goal(wrapper);
    split.addRecipient(&compute_goal);

    /* DepthViewer for visualizing the body tracking */
    std::vector<Eigen::MatrixXd> vec;
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    vec.push_back(identity);
    MultiDepthViewer depth_viewer(vec);
    split.addRecipient(&depth_viewer);

    /* TODO: send a movebase goal over ros */

    ros::init(argc, argv, "body_tracking_node");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseGoal>("person_location", 1000);

    // MoveBaseClient ac("move_base", true);

    // // wait for the action server to come up
    // while(!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    move_base_msgs::MoveBaseGoal goal;


    ros::Rate loop_rate(15);
    while (ros::ok()) {
        wrapper.doOnce();
        ros::spinOnce();

        float* position = compute_goal.getPosition();
        float* orientation = compute_goal.getOrientation();

        goal.target_pose.pose.position.x = position[0];
        goal.target_pose.pose.position.y = position[1];
        goal.target_pose.pose.position.z = position[2];

        goal.target_pose.pose.orientation.x = orientation[0];
        goal.target_pose.pose.orientation.y = orientation[1];
        goal.target_pose.pose.orientation.z = orientation[2];
        goal.target_pose.pose.orientation.w = orientation[3];

        goal_pub.publish(goal);
        // ac.sendGoal(goal);
        // ac.waitForResult();
        // ROS_INFO("x: %.3f\n", goal.target_pose.pose.position.x);
        // ROS_INFO("y: %.3f\n", goal.target_pose.pose.position.y);
        // ROS_INFO("z: %.3f\n\n", goal.target_pose.pose.position.z);

        loop_rate.sleep();
    }

    return 0;
}
