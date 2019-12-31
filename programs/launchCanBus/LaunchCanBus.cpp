// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchCanBus.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <ColorDebug.h>

using namespace roboticslab;

/************************************************************************/

bool LaunchCanBus::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("%s\n", rf.toString().c_str());

    if (rf.check("help"))
    {
        std::printf("LaunchCanBus options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\t--mode [pos]\t--homePoss\n\n");
        CD_DEBUG_NO_HEADER("%s\n", rf.toString().c_str());
        return false;
    }

    yarp::os::Property robotConfig;
    const auto * robotConfigPtr = &robotConfig;
    std::string configPath = rf.findFileByName("config.ini");

    if (configPath.empty() || !robotConfig.fromConfigFile(configPath))
    {
        CD_WARNING("Config file \"%s\" not found or unsufficient permissions.\n", configPath.c_str());
    }

    CD_DEBUG("config.ini: %s\n", robotConfig.toString().c_str());

    yarp::conf::vocab32_t mode = rf.check("mode", yarp::os::Value(VOCAB_CM_POSITION), "initial mode of operation").asVocab();
    bool homing = rf.check("home", "perform initial homing procedure");

    yarp::os::Bottle devCan = rf.findGroup("devCan", "CAN controlboard devices").tail();
    yarp::os::Bottle wrapper = rf.findGroup("wrapper", "YARP wrappers devices").tail();

    if (devCan.isNull() || devCan.size() == 0)
    {
        CD_ERROR("Missing or empty \"devCan\" section collection.\n");
        return false;
    }

    if (wrapper.isNull() || wrapper.size() == 0)
    {
        CD_ERROR("Missing or empty \"wrapper\" section collection.\n");
        return false;
    }

    // CAN devices

    for (int i = 0; i < devCan.size(); i++)
    {
        std::string canDeviceLabel = devCan.get(i).asString();
        const yarp::os::Bottle & canDeviceGroup = rf.findGroup(canDeviceLabel);

        if (canDeviceGroup.isNull())
        {
            CD_ERROR("Missing CAN device group %s.\n", canDeviceLabel.c_str());
            return false;
        }

        yarp::os::Property canDeviceOptions;
        canDeviceOptions.fromString(canDeviceGroup.toString());
        canDeviceOptions.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));
        canDeviceOptions.put("home", homing);

        yarp::dev::PolyDriver * canDevice = new yarp::dev::PolyDriver;
        canDevices.push(canDevice, canDeviceLabel.c_str());

        if (!canDevice->open(canDeviceOptions))
        {
            CD_ERROR("CAN device %s configuration failure.\n", canDeviceLabel.c_str());
            return false;
        }
    }

    // network wrappers

    for (int i = 0; i < wrapper.size(); i++)
    {
        std::string wrapperDeviceLabel = wrapper.get(i).asString();
        const yarp::os::Bottle & wrapperDeviceGroup = rf.findGroup(wrapperDeviceLabel);

        if (wrapperDeviceGroup.isNull())
        {
            CD_ERROR("Missing wrapper device group %s.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        yarp::os::Property wrapperDeviceOptions;
        wrapperDeviceOptions.fromString(wrapperDeviceGroup.toString());
        wrapperDeviceOptions.unput("calibrator"); // custom property added by us

        yarp::dev::PolyDriver * wrapperDevice = new yarp::dev::PolyDriver;
        wrapperDevices.push(wrapperDevice, wrapperDeviceLabel.c_str());

        if (!wrapperDevice->open(wrapperDeviceOptions))
        {
            CD_ERROR("Wrapper device %s configuration failure.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        yarp::dev::IMultipleWrapper * iMultipleWrapper;

        if (!wrapperDevice->view(iMultipleWrapper))
        {
            CD_ERROR("Unable to view IMultipleWrapper in %s.\n", wrapperDeviceLabel.c_str());
            return false;
        }

        yarp::dev::PolyDriverList temp;
        temp = canDevices;

        std::string calibratorDeviceLabel = wrapperDeviceGroup.find("calibrator").asString();

        if (!calibratorDeviceLabel.empty())
        {
            if (robotConfig.toString().empty())
            {
                CD_WARNING("Missing robot config, but calibrator device was requested.\n");
                goto attachToWrapper; // ave Satanas
            }

            yarp::os::Bottle & calibratorDeviceGroup = robotConfig.findGroup(calibratorDeviceLabel);

            if (calibratorDeviceGroup.isNull())
            {
                CD_WARNING("Missing calibrator device group %s.\n", calibratorDeviceLabel.c_str());
                goto attachToWrapper; // ave Satanas
            }

            yarp::os::Property calibratorDeviceOptions;
            calibratorDeviceOptions.fromString(calibratorDeviceGroup.toString());
            calibratorDeviceOptions.put("robotConfig", yarp::os::Value::makeBlob(&robotConfigPtr, sizeof(robotConfigPtr)));
            calibratorDeviceOptions.put("joints", wrapperDeviceOptions.find("joints"));

            yarp::dev::PolyDriver * calibratorDevice = new yarp::dev::PolyDriver;
            yarp::dev::PolyDriverDescriptor descriptor(calibratorDevice, "calibrator"); // key name enforced by CBW2::attachAll()
            calibratorDevices.push(descriptor);

            if (!calibratorDevice->open(calibratorDeviceOptions))
            {
                CD_ERROR("Calibrator device %s configuration failure.\n", calibratorDeviceLabel.c_str());
                return false;
            }

            yarp::dev::IRemoteCalibrator * iRemoteCalibrator;

            if (!calibratorDevice->view(iRemoteCalibrator))
            {
                CD_ERROR("Unable to view IRemoteCalibrator in calibrator device %s.\n", calibratorDeviceLabel.c_str());
                return false;
            }

            yarp::dev::IWrapper * iWrapper;

            if (!calibratorDevice->view(iWrapper))
            {
                CD_ERROR("Unable to view IWrapper in calibrator device %s.\n", calibratorDeviceLabel.c_str());
                return false;
            }

            if (!iWrapper->attach(wrapperDevice))
            {
                CD_ERROR("Unable to attach calibrator to wrapper device %s.\n", wrapperDeviceLabel.c_str());
                return false;
            }

            temp.push(descriptor);
        }

        attachToWrapper:
        if (!iMultipleWrapper->attachAll(temp))
        {
            CD_ERROR("Unable to attach wrapper %s to CAN devices.\n", wrapperDeviceLabel.c_str());
            return false;
        }
    }

    // initial control modes

    for (int i = 0; i < canDevices.size(); i++)
    {
        yarp::dev::IControlMode * iControlMode;
        yarp::dev::IEncoders * iEncoders;

        if (!canDevices[i]->poly->view(iControlMode))
        {
            CD_ERROR("Unable to view IControlMode in %s.\n", canDevices[i]->key.c_str());
            return false;
        }

        if (!canDevices[i]->poly->view(iEncoders))
        {
            CD_ERROR("Unable to view IEncoders in %s.\n", canDevices[i]->key.c_str());
            return false;
        }

        int axes;

        if (!iEncoders->getAxes(&axes))
        {
            CD_ERROR("Unable to retrieve axes in %s.\n", canDevices[i]->key.c_str());
            return false;
        }

        std::vector<yarp::conf::vocab32_t> modes(axes, mode);

        if (!iControlMode->setControlModes(modes.data()))
        {
            CD_ERROR("Unable to set %s mode in %s.\n", yarp::os::Vocab::decode(mode).c_str(), canDevices[i]->key.c_str());
            return false;
        }

        CD_SUCCESS("Set %s mode in %s.\n", yarp::os::Vocab::decode(mode).c_str(), canDevices[i]->key.c_str());
    }

    // homing on start

    if (homing)
    {
        if (calibratorDevices.size() == 0)
        {
            CD_ERROR("Homing procedure requested, but no calibrator devices loaded.\n");
            return false;
        }

        for (int i = 0; i < calibratorDevices.size(); i++)
        {
            yarp::dev::IRemoteCalibrator * iRemoteCalibrator;
            calibratorDevices[i]->poly->view(iRemoteCalibrator);

            if (!iRemoteCalibrator->homingWholePart())
            {
                CD_ERROR("Homing procedure failed for calibrator id %d.\n", i);
                return false;
            }
        }
    }

    return true;
}

/************************************************************************/

bool LaunchCanBus::updateModule()
{
    return true;
}

/************************************************************************/

double LaunchCanBus::getPeriod()
{
    return 3.0;
}

/************************************************************************/

bool LaunchCanBus::close()
{
    for (int i = 0; i < calibratorDevices.size(); i++)
    {
        if (!calibratorDevices[i]->poly->close())
        {
            CD_WARNING("Device %s did not close properly.\n", calibratorDevices[i]->key.c_str());
        }

        delete calibratorDevices[i]->poly;
    }

    for (int i = 0; i < wrapperDevices.size(); i++)
    {
        if (!wrapperDevices[i]->poly->close())
        {
            CD_WARNING("Device %s did not close properly.\n", wrapperDevices[i]->key.c_str());
        }

        delete wrapperDevices[i]->poly;
    }

    for (int i = 0; i < canDevices.size(); i++)
    {
        if (!canDevices[i]->poly->close())
        {
            CD_WARNING("Device %s did not close properly.\n", canDevices[i]->key.c_str());
        }

        delete canDevices[i]->poly;
    }

    return true;
}

/************************************************************************/
