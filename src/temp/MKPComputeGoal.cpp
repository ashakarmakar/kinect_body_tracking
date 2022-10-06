#include "include/KinectWrapper.h"
#include "include/MKConfig.h"
#include "include/MultiKinectPacket.h"
#include "include/MKPRecipient.h"
#include "include/MultiKinectWrapper.h"
#include "include/MKPComputeGoal.h"
#include <cstdlib>
// for polyfit
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <Eigen/QR>

MKPComputeGoal::MKPComputeGoal(MultiKinectWrapper &mkw) : _mkw(mkw) {

}

MKPComputeGoal::~MKPComputeGoal() {
    delete[] position;
    delete[] orientation;
}

void MKPComputeGoal::addRecipient(MKPRecipient *r) {
    _r.push_back(r);
}

/* get the body frame from the packet and fill out the position and orientation arrays with the goal */
void MKPComputeGoal::receiveFrame(MultiKinectPacket &mkp) {
    PROFILE_START;

    body_frame = mkp[0].getBodyFrame();
    if (body_frame) computeGoal();
    
    PROFILE_END("MKPComputeGoal.receiveFrame");
}

float* MKPComputeGoal::getPosition() {
    return position;
    
}
float* MKPComputeGoal::getOrientation() {
    return orientation;
}

void MKPComputeGoal::scanline() {
    std::vector<double> xpos;
    std::vector<double> zpos;
    std::vector<double> coeff;

    // get fitted line
    for (int i = 0; i < body_frame.get_num_bodies(); i++) {
        k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(i);

        position[0] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.x / 1000.0 * 3.281;
        position[1] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.y / 1000.0 * 3.281;
        position[2] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.z / 1000.0 * 3.281;

        xpos.push_back(position[0]);
        zpos.push_back(position[2]);
    }

    /* 
        1. save the skeleton of the last person
        2. save their x value
    */

    /* define a threshold value - if there is anyone within this distance, they are considered near*/
    double distance_threshold = 3.0;
    std::vector<k4abt_skeleton_t> end_people;
    
    // Go through each body in line
    for (int i = 0; i < body_frame.get_num_bodies(); i++) {
        k4abt_skeleton_t person1 = body_frame.get_body_skeleton(i);
        int num_people_within = 0;

        // get current person's x and z pos
        double person1x = person1.joints[K4ABT_JOINT_PELVIS].position.xyz.x / 1000.0 * 3.281;
        double person1z = person1.joints[K4ABT_JOINT_PELVIS].position.xyz.z / 1000.0 * 3.281;

        // compare current body with all other bodies in frame to check how many are near this body
        for (int j = i+1; j < body_frame.get_num_bodies(); j++) {
            k4abt_skeleton_t person2 = body_frame.get_body_skeleton(j);

            // get comparing person's x and z pos
            double person2x = person2.joints[K4ABT_JOINT_PELVIS].position.xyz.x / 1000.0 * 3.281;
            double person2z = person2.joints[K4ABT_JOINT_PELVIS].position.xyz.z / 1000.0 * 3.281;

            // use distance formula to get absolute difference between two people
            double difference = sqrt(pow(person1x-person2x, 2) +  pow(person1z-person2z, 2));

            // if person2 is considered to be near person1, increment num_people_within
            if (difference <= threshold) {
                num_people_within++;
            }
        }

        // if none or only 1 person is near the current person, that means they are at the front or back of line
        if (num_people_within <= 1) {
            end_people.push_back(person1);
        }
    }

    k4abt_skeleton_t last_person;
    // now that we have end people, calculate direction line is facing
    if (end_people.size() == 0) {
        printf("error! couldn't find end people!\n");
    } else if (end_people.size() == 1) {
        last_person = end_people.pop_back();
    } else if (end_people.size() > 2) {
        printf("error! detected more than 2 end people!\n");
    } else {
        k4abt_skeleton_t person1 = end_people.pop_back();
        k4abt_skeleton_t person2 = end_people.pop_back();

        double person1_orientation = person1.joints[K4ABT_JOINT_PELVIS].position.xyz.z / 1000.0 * 3.281;
        double person2_orientation = person2.joints[K4ABT_JOINT_PELVIS].position.xyz.z / 1000.0 * 3.281;

        if () {

        } else {
            
        }

    }

    // get fitted function
    polyfit(xpos, zpos, coeff, 1);
    std::vector<double> fitted_z_pos ;
    for (int i = 0; i < xpos.size(); i++) {
        double zfitted = coeff[0] + coeff[1]*xpos.at(i) + coeff[2]*(pow(xpos.at(i), 2)) +coeff[3]*(pow(xpos.at(i), 3));
        fitted_z_pos.push_back(zfitted);
    }
}

/* fills out the position and orientation arrays with the person's position and orientation. */
void MKPComputeGoal::computeGoal() {
    k4abt_skeleton_t skeleton = body_frame.get_body_skeleton(0);

    position[0] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.x / 1000.0 * 3.281;
    position[1] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.y / 1000.0 * 3.281;
    position[2] = skeleton.joints[K4ABT_JOINT_PELVIS].position.xyz.z / 1000.0 * 3.281;

    orientation[0] = skeleton.joints[K4ABT_JOINT_PELVIS].orientation.wxyz.w / 1000.0 * 3.281;
    orientation[1] = skeleton.joints[K4ABT_JOINT_PELVIS].orientation.wxyz.x / 1000.0 * 3.281;
    orientation[2] = skeleton.joints[K4ABT_JOINT_PELVIS].orientation.wxyz.y / 1000.0 * 3.281;
    orientation[3] = skeleton.joints[K4ABT_JOINT_PELVIS].orientation.wxyz.z / 1000.0 * 3.281;

    // printf("%.3f\n", position[1]);

    // printf("position: %.3f, %.3f, %.3f\n", position[0], position[1], position[2]);
    // printf("orientation: %.3f, %.3f, %.3f, %.3f\n", orientation[0], orientation[1],orientation[2], orientation[3]);
}

/* 
1. scan line, determine average distance between each person
2. determine who is at the end of the line
    a) no one on one side of them
    b) if front and back angle < 0, person on the right. vice versa
3. calculate linear regression + find expected value for last person
4. move to (expected value) + (average x distance) + (average y distance).

*/

//  calculate the expected position of the last person in line
void MKPComputeGoal::polyfit(const std::vector<double> &t,const std::vector<double> &v, std::vector<double> &coeff, int order)
{
	// Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	Eigen::MatrixXd T(t.size(), order + 1);
	Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	Eigen::VectorXd result;

	// check to make sure inputs are correct
	assert(t.size() == v.size());
	assert(t.size() >= order + 1);
	// Populate the matrix
	for(size_t i = 0 ; i < t.size(); ++i)
	{
		for(size_t j = 0; j < order + 1; ++j)
		{
			T(i, j) = pow(t.at(i), j);
		}
	}
	
	// Solve for linear least square fit
	result  = T.householderQr().solve(V);
	coeff.resize(order+1);
	for (int k = 0; k < order+1; k++)
	{
		coeff[k] = result[k];
	}

}