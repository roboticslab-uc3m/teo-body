// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setLimitsRaw(int axis, double min, double max)
{
    yCTrace(DEXTRA, "%d %f %f", axis, min, max);
    CHECK_JOINT(axis);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getLimitsRaw(int axis, double * min, double * max)
{
    yCTrace(DEXTRA, "%d", axis);
    CHECK_JOINT(axis);

    auto limits = Synapse::LIMITS[axis];
    *min = limits.first;
    *max = limits.second;

    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setVelLimitsRaw(int axis, double min, double max)
{
    yCWarning(DEXTRA, "setVelLimitsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getVelLimitsRaw(int axis, double * min, double * max)
{
    yCWarning(DEXTRA, "getVelLimitsRaw() not supported");
    return false;
}

// -----------------------------------------------------------------------------
