#ifndef MKPR_COMPUTE_GOAL_H
#define MKPR_COMPUTE_GOAL_H

#include "MKPRecipient.h"

#include <k4abt.hpp>

class MKPRecipient;
class MultiKinectWrapper;

class MKPComputeGoal : public MKPRecipient {
public:
    MKPComputeGoal(MultiKinectWrapper &mkw);
    ~MKPComputeGoal();

    void addRecipient(MKPRecipient *r);

    void receiveFrame(MultiKinectPacket &mkp);
    /* add a method to do the bulk of the computing: take the body position data and compute where the robot should go relative to the body
        add a get() method that will allow the main method to access that data
        publish that data to a ROS topic */
    
    float* getPosition();
    float* getOrientation();
    void computeGoal();
    void polyfit(const std::vector<double> &t,const std::vector<double> &v, std::vector<double> &coeff, int order);
    void scanline();

    k4abt::frame body_frame;   

    
protected:
    MultiKinectWrapper &_mkw;
    std::vector<MKPRecipient *> _r;
    
    // k4abt_tracker_configuration_t _tracker_config;
    // k4abt::tracker *_tracker;

    float position [3];
    float orientation [4];
};

#endif