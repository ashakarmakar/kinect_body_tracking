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
    // std::vector<Eigen::MatrixXd> vec;
    // const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    // vec.push_back(identity);
    // MultiDepthViewer depth_viewer(vec);
    // split.addRecipient(&depth_viewer);

    /* TODO: send a movebase goal over ros */

    ros::init(argc, argv, "body_tracking_node");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseGoal>("person_location", 1000);

    tf::TransformBroadcaster broadcaster;

    // MoveBaseClient ac("/move_base/move", true);

    // // wait for the action server to come up
    // while(!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped pose;

    ros::Rate loop_rate(15);
    while (ros::ok()) {
        wrapper.doOnce();
        ros::spinOnce();

        // float* position = compute_goal.getPosition();
        // float* orientation = compute_goal.getOrientation();

        // pose.pose[i]->position.x = position[0];
        // pose.pose[i]->position.y = position[1];
        // pose.pose[i]->position.z = position[2];

        // pose.pose->orientation.x = orientation[0];
        // pose.pose->orientation.y = orientation[1];
        // pose.pose->orientation.z = orientation[2];
        // pose.pose->orientation.w = orientation[3];

        // goal.target_pose = pose;

        // printf("body 0: position x: %.7f\n", compute_goal.bodies[0]->position.x);
        // printf("body 1: position x: %.7f\n", compute_goal.bodies[1]->position.x);


        // ROS_INFO("number of bodies: %d\n", compute_goal.getNumBodies());
        //printf("NODE: body %d x: %.5f\n", i, compute_goal.bodies[i]->x);

        // MKPComputeGoal::Pose** tempBodies = compute_goal.bodies;
        printf("NODE: body %d x ptr: %p\n", 0, compute_goal.bodies[0]);
        printf("NODE: body %d x ptr: %p\n", 1, compute_goal.bodies[1]);

        for (int i = 0; i < compute_goal.getNumBodies(); i++) {
            // ROS_INFO("body %d: position y: %.3f\n", i, compute_goal.bodies[i]->position.y);
            // ROS_INFO("body %d: position z: %.3f\n", i, compute_goal.bodies[i]->position.z);

        //     broadcaster.sendTransform( 
        //             tf::StampedTransform(tf::Transform(tf::Quaternion(compute_goal.bodies[i]->orientation.x, compute_goal.bodies[i]->orientation.y, 
        //                                                                 compute_goal.bodies[i]->orientation.z, compute_goal.bodies[i]->orientation.w), 
        //                                                 tf::Vector3(compute_goal.bodies[i]->position.x, compute_goal.bodies[i]->position.y, compute_goal.bodies[i]->position.z)), 
        //                                                 ros::Time::now(), "map", "line_person  "+ i));

        }

        // Pose** bodies = compute_goal.getBodies();



        // ac.sendGoal(goal);
        // ac.waitForResult();
        loop_rate.sleep();
    }

    return 0;
}
