// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

// ------------------- IControlLimits related ------------------------------------

bool roboticslab::AmorControlboard::setLimits(int axis, double min, double max)
{
    yError("setLimits() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getLimits(int axis, double *min, double *max)
{
    yTrace("%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        yError("amor_get_joint_info() failed: %s", amor_error());
        return false;
    }

    *min = toDeg(parameters.lowerJointLimit);
    *max = toDeg(parameters.upperJointLimit);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setVelLimits(int axis, double min, double max)
{
    yError("setVelLimits() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getVelLimits(int axis, double *min, double *max)
{
    yTrace("%d", axis);

    if (!indexWithinRange(axis))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, axis, &parameters) != AMOR_SUCCESS)
    {
        yError("amor_get_joint_info() failed: %s", amor_error());
        return false;
    }

    *max = toDeg(parameters.maxVelocity);
    *min = -(*max);

    return true;
}

// -----------------------------------------------------------------------------
