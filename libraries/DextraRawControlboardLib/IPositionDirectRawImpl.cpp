// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ----------------------------------------------------------------------------------------

bool DextraRawControlboard::setPositionRaw(int j, double ref)
{
    yCTrace(DEXTRA, "%d %f", j, ref);
    return positionMoveRaw(j, ref);
}

// ----------------------------------------------------------------------------------------

bool DextraRawControlboard::setPositionsRaw(const double * refs)
{
    return positionMoveRaw(refs);
}

// ----------------------------------------------------------------------------------------

bool DextraRawControlboard::setPositionsRaw(int n_joint, const int * joints, const double * refs)
{
    return positionMoveRaw(n_joint, joints, refs);
}

// ----------------------------------------------------------------------------------------
