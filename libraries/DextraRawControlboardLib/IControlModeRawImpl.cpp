// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <yarp/os/Log.h>
#include <yarp/os/Vocab.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getControlModeRaw(int j, int * mode)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    *mode = VOCAB_CM_POSITION;
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getControlModesRaw(int * modes)
{
    yTrace("");

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
    yTrace("%d", n_joint);

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
    yTrace("%d %s", j, yarp::os::Vocab::decode(mode).c_str());
    CHECK_JOINT(j);
    return false; // don't allow control modes other than position direct, for onw
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::setControlModesRaw(int * modes)
{
    yTrace("");

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
    yTrace("%d", n_joint);

    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setControlModeRaw(joints[i], modes[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------
