#include "include/MKPPersonTracker.h"
#include "include/KinectWrapper.h"
#include "include/MKConfig.h"
#include "include/MultiKinectPacket.h"
#include "include/MKPRecipient.h"
#include "include/MultiKinectWrapper.h"

#include <cstdlib>

MKPPersonTracker::MKPPersonTracker(MultiKinectWrapper &mkw) : _mkw(mkw), _tracker_config(K4ABT_TRACKER_CONFIG_DEFAULT) {
    size_t nCameras = _mkw.getConfig()._nCameras; 
    _tracker = new k4abt::tracker[nCameras];
    for(size_t i = 0; i < nCameras; i++) {
        _tracker[i] = k4abt::tracker::create(_mkw.getKinectWrapper(i)->getCalibration(),_tracker_config);
    }
}

MKPPersonTracker::~MKPPersonTracker() {
    delete[] _tracker;
}

void MKPPersonTracker::addRecipient(MKPRecipient *r) {
    _r.push_back(r);
}

void MKPPersonTracker::receiveFrame(MultiKinectPacket &mkp) {
    PROFILE_START;
    size_t nCameras = _mkw.getConfig()._nCameras; 

    std::chrono::milliseconds wait = std::chrono::milliseconds(K4A_WAIT_INFINITE);

    for(size_t i = 0; i < nCameras; i++) {
        _tracker[i].enqueue_capture(mkp[i].getCapture(), wait);
    }
    for(size_t i = 0; i < nCameras; i++) {
        if (mkp[i].getBodyFrame()) {
            k4abt::frame bf;
            _tracker[i].pop_result(&bf, wait);
            mkp[i].setBodyFrame(bf);
        } else {
            printf("no body in frame.\n");
        }
    }
    PROFILE_END("MKPPersonTracker.receiveFrame");

    for(size_t i = 0; i < _r.size(); i++) {
        _r[i]->receiveFrame(mkp);
    }
}