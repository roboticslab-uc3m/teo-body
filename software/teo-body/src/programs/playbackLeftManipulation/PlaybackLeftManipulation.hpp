// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK_LEFT_MANIPULATION__
#define __PLAYBACK_LEFT_MANIPULATION__

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
#define DEFAULT_FILE_NAME "leftManipulationRecording.txt"

#define DEFAULT_PT_MODE_MS 50

using namespace yarp::os;
using namespace yarp::dev;

/**
 * @ingroup playbackLeftManipulation
 *
 * The PlaybackLeftManipulation class tests the class as a controlboard.
 * 
 */
class PlaybackLeftManipulation : public RFModule {

    public:
        PlaybackLeftManipulation();
        bool configure(ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver leftArmDevice;

        yarp::dev::PolyDriver leftGripperDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        PlaybackThread playbackThread;

};

#endif  // __PLAYBACK_LEFT_MANIPULATION__
