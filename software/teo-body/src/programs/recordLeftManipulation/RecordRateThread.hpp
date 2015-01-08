// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RECORD_LEFT_RATE_THREAD__
#define __RECORD_LEFT_RATE_THREAD__

#include <vector>

#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#define DEFAULT_MS 50  // [ms], overwritten by parent DEFAULT_PT_MODE_MS.

/**
 * @ingroup recordLeftManipulation
 *
 * The RecordRateThread class tests the class as a controlboard.
 *
 */
class RecordRateThread : public yarp::os::RateThread {

    public:
        // Set the Thread Rate in the class constructor
        RecordRateThread() : RateThread(DEFAULT_MS) {}  // In ms

        /**
         * Loop function. This is the thread itself.
         */
        virtual void run();

        void setFilePtr(FILE *value);

        yarp::dev::IEncoders *leftArmEnc;
        yarp::dev::ITorqueControl *leftArmTrq;

        yarp::dev::IEncoders *leftGripperEnc;

        int leftArmNumMotors;

    protected:

        FILE * filePtr;


};

#endif  // __RECORD_LEFT_RATE_THREAD__

