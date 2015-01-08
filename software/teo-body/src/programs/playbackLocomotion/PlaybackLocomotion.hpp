// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK_MANIPULATION__
#define __PLAYBACK_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>
#include <stdlib.h>  // Used for ::getenv

#include "ColorDebug.hpp"

#include "PlaybackThread.hpp"

#define DEFAULT_ROBOT_NAME "/teo"
#define DEFAULT_FILE_NAME "locomotionRecording.txt"

#define DEFAULT_PT_MODE_MS 50

using namespace yarp::os;
using namespace yarp::dev;

/**
 * @ingroup playbackLocomotion
 *
 * The PlaybackLocomotion class tests the class as a controlboard.
 * 
 */
class PlaybackLocomotion : public RFModule {

    public:
        PlaybackLocomotion();
        bool configure(ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver leftLegDevice;
        yarp::dev::PolyDriver rightLegDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        PlaybackThread playbackThread;

};

#endif  // __PLAYBACK_MANIPULATION__

