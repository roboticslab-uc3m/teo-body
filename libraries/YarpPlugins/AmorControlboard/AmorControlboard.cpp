// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::indexWithinRange(const int& idx)
{
    if (idx >= AMOR_NUM_JOINTS)
    {
        yCError(AMOR, "Index out of range: %d >= %d", idx, AMOR_NUM_JOINTS);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::batchWithinRange(const int& n_joint)
{
    if (n_joint == 0)
    {
        yCWarning(AMOR, "Passed array of size (n_joint) equal to zero");
        return true;
    }

    if (n_joint < 0 || n_joint > AMOR_NUM_JOINTS)
    {
        yCError(AMOR, "n_joint out of range (< 0 or > %d): %d", AMOR_NUM_JOINTS, n_joint);
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

double roboticslab::AmorControlboard::toDeg(double rad)
{
    return rad * 180 / M_PI;
}

// -----------------------------------------------------------------------------

double roboticslab::AmorControlboard::toRad(double deg)
{
    return deg * M_PI / 180;
}

// -----------------------------------------------------------------------------
