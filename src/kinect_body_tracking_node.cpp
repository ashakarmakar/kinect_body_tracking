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
#include <geometry_msgs/PoseArray.h>

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
    ros::Publisher line_pub = nh.advertise<geometry_msgs::PoseArray>("fitted_line", 1000);

    tf::TransformBroadcaster broadcaster;
    tf::TransformBroadcaster static_broadcaster;

    // MoveBaseClient ac("/move_base/move", true);

    // // wait for the action server to come up
    // while(!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }

    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseArray fittedLine;

    ros::Rate loop_rate(15);
    while (ros::ok()) {
        wrapper.doOnce();

        // goal.target_pose = pose;
        
        geometry_msgs::TransformStamped static_transformStamped;        
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "map";
        static_transformStamped.child_frame_id = "camera";

        static_transformStamped.transform.translation.x = 0.0;
        static_transformStamped.transform.translation.y = 0.0;
        static_transformStamped.transform.translation.z = 1.0;

        static_transformStamped.transform.rotation.x = -0.5;
        static_transformStamped.transform.rotation.y = 0.5;
        static_transformStamped.transform.rotation.z = -0.5;
        static_transformStamped.transform.rotation.w = 0.5;

        static_broadcaster.sendTransform(static_transformStamped);

        if (compute_goal.coeffs.size() > 0) {    
            /* compute the fitted points and add to a temp array*/        
            geometry_msgs::Pose fittedPoints[100]; //compute_goal.fittedPoints;
            geometry_msgs::Pose pt;
            
            int index = 0;
            for (double i = 0.0; i < 10.0; i += 0.2) {
                // printf("\ncalculating xfitted\n");
                double xfitted = compute_goal.coeffs[0] + compute_goal.coeffs[1]*i;// + compute_goal.coeffs[2]*(pow(i, 2));
                pt.position.x = xfitted;
                pt.position.y = 0.0; // camera level
                pt.position.z = i;

                pt.orientation.w = 1;
                pt.orientation.x = 0;
                pt.orientation.y = 0;
                pt.orientation.z = 0;

                fittedPoints[index] = pt;
                //printf("\nindex: %d\n", index);
                index++;
            }
            
            /* create a vector from the temp array */
            std::vector<geometry_msgs::Pose> point_vector (fittedPoints, fittedPoints + sizeof(fittedPoints) / sizeof(geometry_msgs::Pose));
            fittedLine.poses.clear();
            
            // loads all the Point32 objects from vector into actual message's point fields
            int i = 0;
            for (std::vector<geometry_msgs::Pose>::iterator it = point_vector.begin(); it != point_vector.end(); ++it) {
                geometry_msgs::Pose point_2;
                point_2.position.x = (*it).position.x;
                point_2.position.y = (*it).position.y;
                point_2.position.z = (*it).position.z;

                point_2.orientation.x = (*it).orientation.w;
                point_2.orientation.x = (*it).orientation.x;
                point_2.orientation.y = (*it).orientation.y;
                point_2.orientation.z = (*it).orientation.z;

                fittedLine.poses.push_back(point_2);
                i++;
            }

            fittedLine.header.seq = 1;
            fittedLine.header.frame_id = "camera";
            fittedLine.header.stamp = ros::Time::now();
            line_pub.publish(fittedLine);
        }

        for (int i = 0; i < compute_goal.getNumBodies(); i++) {
            broadcaster.sendTransform( 
                    tf::StampedTransform(tf::Transform(tf::Quaternion(compute_goal.bodies[i]->orientation->x, compute_goal.bodies[i]->orientation->y, 
                                                                        compute_goal.bodies[i]->orientation->z, compute_goal.bodies[i]->orientation->w), 
                                                        tf::Vector3(compute_goal.bodies[i]->position->x, compute_goal.bodies[i]->position->y, compute_goal.bodies[i]->position->z)), 
                                                        ros::Time::now(), "camera", "person_"+ std::to_string(i)));
        }

        // ac.sendGoal(goal);
        // ac.waitForResult();
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
