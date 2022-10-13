#include <MultiKinectWrapper.h>
#include <MKConfig.h>
#include "AsyncMKPRSplitter.h"
#include "MKPPersonTracker.h"
#include "MKPComputeGoal.h"
#include "DepthViewer.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

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

    tf::TransformBroadcaster broadcaster;

    MoveBaseClient ac("/move_base/move", true);

    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped pose;

    ros::Rate loop_rate(15);
    while (ros::ok()) {
        wrapper.doOnce();
        ros::spinOnce();

        float* position = compute_goal.getPosition();
        float* orientation = compute_goal.getOrientation();

        pose.pose.position.x = position[0];
        pose.pose.position.y = position[1];
        pose.pose.position.z = position[2];

        pose.pose.orientation.x = orientation[0];
        pose.pose.orientation.y = orientation[1];
        pose.pose.orientation.z = orientation[2];
        pose.pose.orientation.w = orientation[3];

        goal.target_pose = pose;

        // broadcaster.sendTransform(
        //                       tf::StampedTransform(tf::Transform(tf::Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]), tf::Vector3(position[0], position[1], position[2])), 
        //                                            ros::Time::now(), 
        //                                            "map", 
        //                                            "person"));

        ac.sendGoal(goal);
        ac.waitForResult();
        loop_rate.sleep();
    }

    return 0;
}
