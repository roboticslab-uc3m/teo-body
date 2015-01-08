// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __RECORD_MANIPULATION__
#define __RECORD_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>
#include <stdlib.h>  // Used for ::getenv

#include <curses.h>

#include "ColorDebug.hpp"

#include "RecordRateThread.hpp"
#include "MoveGripperThread.hpp"


#define DEFAULT_ROBOT_NAME "/teo"
#define DEFAULT_FILE_NAME "manipulationRecording.txt"
#define DEFAULT_PT_MODE_MS 50

/**
 * @ingroup recordManipulation
 *
 * The RecordManipulation class tests the class as a controlboard.
 * 
 */
class RecordManipulation : public yarp::os::RFModule {

    public:
        RecordManipulation();
        bool configure(yarp::os::ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver leftArmDevice;
        yarp::dev::PolyDriver rightArmDevice;

        yarp::dev::PolyDriver leftGripperDevice;
        yarp::dev::PolyDriver rightGripperDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        FILE * filePtr;

        RecordRateThread recordRateThread;

        MoveGripperThread moveGripperThread;

};

#endif  // __RECORD_MANIPULATION__

