// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getControlModeRaw(int j, int * mode)
{
    yCTrace(DEXTRA, "%d", j);
    CHECK_JOINT(j);
    *mode = VOCAB_CM_POSITION;
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getControlModesRaw(int * modes)
{
    yCTrace(DEXTRA, "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getControlModeRaw(j, &modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getControlModesRaw(int n_joint, const int * joints, int * modes)
{
    yCTrace(DEXTRA, "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getControlModeRaw(joints[i], &modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setControlModeRaw(int j, int mode)
{
#if YARP_VERSION_MINOR >= 5
    yCTrace(DEXTRA, "%d %s", j, yarp::os::Vocab32::decode(mode).c_str());
#else
    yCTrace(DEXTRA, "%d %s", j, yarp::os::Vocab::decode(mode).c_str());
#endif
    CHECK_JOINT(j);
    return false; // don't allow control modes other than position direct, for onw
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setControlModesRaw(int * modes)
{
    yCTrace(DEXTRA, "");

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= setControlModeRaw(j, modes[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setControlModesRaw(int n_joint, const int * joints, int * modes)
{
    yCTrace(DEXTRA, "%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setControlModeRaw(joints[i], modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
