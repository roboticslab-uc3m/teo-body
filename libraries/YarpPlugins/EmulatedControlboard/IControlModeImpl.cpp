// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlboard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

// ------------------- IControlMode Related ------------------------------------

bool roboticslab::EmulatedControlboard::getControlMode(int j, int *mode)
{
    yTrace("%d", j);

    if (controlMode == POSITION_MODE)
    {
        *mode = VOCAB_CM_POSITION;
    }
    else if (controlMode == VELOCITY_MODE)
    {
        *mode = VOCAB_CM_VELOCITY;
    }
    else if (controlMode == POSITION_DIRECT_MODE)
    {
        *mode = VOCAB_CM_POSITION_DIRECT;
    }
    else
    {
        yError("Currently unsupported mode: %s", yarp::os::Vocab::decode(controlMode).c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getControlModes(int *modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getControlMode(i, &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::getControlModes(const int n_joint, const int *joints, int *modes)
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlMode(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setControlMode(const int j, const int mode)
{
    yTrace("%d %s", j, yarp::os::Vocab::decode(mode).c_str());

    if ((unsigned int)j > axes)
    {
        yError("Axis index greater than number of axes (%d > %d)", j, axes);
        return false;
    }

    switch (mode)
    {
    case VOCAB_CM_POSITION:
        return setPositionMode(j);
    case VOCAB_CM_VELOCITY:
        return setVelocityMode(j);
    case VOCAB_CM_TORQUE:
        return setTorqueMode(j);
    case VOCAB_CM_POSITION_DIRECT:
        return setPositionDirectMode(j);
    default:
        return false;
    }
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setControlModes(const int n_joint, const int *joints, int *modes)
{
    bool ok = true;

    for (int j = 0; j < n_joint; j++)
    {
        ok &= setControlMode(joints[j], modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::EmulatedControlboard::setControlModes(int *modes)
{
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= setControlMode(i, modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
