// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusLauncher.hpp"

#include <cstdio>
#include <string>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <ColorDebug.h>

using namespace roboticslab;

/************************************************************************/

bool CanBusLauncher::configure(yarp::os::ResourceFinder &rf)
{
    if (rf.check("help"))
    {
        std::printf("CanBusLauncher options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\t--homePoss\t--externalEncoderWait [s]\n\n");
        std::printf("Note: if the Absolute Encoder doesn't respond, use --externalEncoderWait [seconds] parameter for using default relative encoder position\n");
        CD_DEBUG_NO_HEADER("%s\n", rf.toString().c_str());
        return false;
    }

    std::string mode = rf.check("mode", yarp::os::Value("position"), "initial mode of operation").asString();
    bool homing = rf.check("homePoss", yarp::os::Value(false), "perform initial homing procedure").asBool();
    int timeEncoderWait = rf.check("externalEncoderWait", yarp::os::Value(0), "wait till absolute encoders are ready").asInt32();

    if (timeEncoderWait != 0)
    {
        CD_INFO("Wait time for Absolute Encoder: %d [s]\n", timeEncoderWait);
    }

    const std::string canDevicePrefix = "devCan";
    int canDeviceId = 0;

    while (true)
    {
        std::ostringstream oss;
        oss << canDevicePrefix << canDeviceId;
        std::string canDeviceLabel = oss.str();

        yarp::os::Bottle canDeviceGroup = rf.findGroup(canDeviceLabel);

        if (canDeviceGroup.isNull())
        {
            break;
        }

        CD_DEBUG("%s\n", canDeviceGroup.toString().c_str());

        yarp::os::Property canDeviceOptions;
        canDeviceOptions.fromString(canDeviceGroup.toString());
        canDeviceOptions.put("mode", mode);
        canDeviceOptions.put("waitEncoder", timeEncoderWait);
        canDeviceOptions.put("home", homing);

        yarp::dev::PolyDriver * canDevice = new yarp::dev::PolyDriver(canDeviceOptions);

        if (!canDevice->isValid())
        {
            CD_ERROR("CAN device %s instantiation failure.\n", canDeviceLabel.c_str());
            return false;
        }

        canDevices.push(canDevice, canDeviceLabel.c_str());
        canDeviceId++;
    }

    if (canDeviceId == 0)
    {
        CD_ERROR("Empty CAN device list.\n");
        return false;
    }

    const std::string wrapperDevicePrefix = "wrapper";
    int wrapperDeviceId = 0;

    while (true)
    {
        std::ostringstream oss;
        oss << wrapperDevicePrefix << wrapperDeviceId;
        std::string wrapperDeviceLabel = oss.str();
        yarp::os::Bottle wrapperDeviceGroup = rf.findGroup(wrapperDeviceLabel);

        if (wrapperDeviceGroup.isNull())
        {
            break;
        }

        CD_DEBUG("%s\n", wrapperDeviceGroup.toString().c_str());

        yarp::os::Property wrapperDeviceOptions;
        wrapperDeviceOptions.fromString(wrapperDeviceGroup.toString());

        yarp::dev::PolyDriver * wrapperDevice = new yarp::dev::PolyDriver(wrapperDeviceOptions);

        if (!wrapperDevice->isValid())
        {
            CD_ERROR("Wrapper device %s instantiation failure.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        wrapperDevices.push(wrapperDevice, wrapperDeviceLabel.c_str());

        yarp::dev::IMultipleWrapper * iMultipleWrapper;

        if (!wrapperDevice->view(iMultipleWrapper))
        {
            CD_ERROR("Unable to view IMultipleWrapper in %s.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        if (!iMultipleWrapper->attachAll(canDevices))
        {
            CD_ERROR("Unable to attach CAN devices in %s.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        wrapperDeviceId++;
    }

    if (wrapperDeviceId == 0)
    {
        CD_ERROR("Empty wrapper device list.\n");
        return false;
    }

    return true;
}

/************************************************************************/

bool CanBusLauncher::updateModule()
{
    return true;
}

/************************************************************************/

double CanBusLauncher::getPeriod()
{
    return 3.0;
}

/************************************************************************/

bool CanBusLauncher::close()
{
    for (int i = 0; i < wrapperDevices.size(); i++)
    {
        delete wrapperDevices[i]->poly;
    }

    for (int i = 0; i < canDevices.size(); i++)
    {
        delete canDevices[i]->poly;
    }

    return true;
}

/************************************************************************/