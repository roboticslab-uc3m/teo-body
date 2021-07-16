// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

// ------------------- IAxisInfo related ------------------------------------

bool roboticslab::AmorControlboard::getAxisName(int axis, std::string& name)
{
    yCTrace(AMOR, "%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    switch (axis)
    {
        case 0:
            name = "A1";
            break;
        case 1:
            name = "A2";
            break;
        case 2:
            name = "A2.5";
            break;
        case 3:
            name = "A3";
            break;
        case 4:
            name = "A4";
            break;
        case 5:
            name = "A5";
            break;
        case 6:
            name = "A6";
            break;
        default:
            yCError(AMOR, "Unrecognized axis: %d", axis);
            return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    yCTrace(AMOR, "%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;

    return true;
}

// -----------------------------------------------------------------------------
