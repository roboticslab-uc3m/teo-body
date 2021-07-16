// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraRawControlboard.hpp"

#include <algorithm>

#include <yarp/os/Log.h>
#include <yarp/os/Time.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// ------------------------------------------------------------------------------

bool DextraRawControlboard::resetEncoderRaw(int j)
{
    yCTrace(DEXTRA, "%d", j);
    return setEncoderRaw(j, 0.0);
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::resetEncodersRaw()
{
    yCTrace(DEXTRA, "");
    Synapse::Setpoints setpoints = {0};
    setSetpoints(setpoints);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::setEncoderRaw(int j, double val)
{
    yCTrace(DEXTRA, "%d %f", j, val);
    CHECK_JOINT(j);
    setSetpoint(j, val);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::setEncodersRaw(const double * vals)
{
    yCTrace(DEXTRA, "");
    Synapse::Setpoints setpoints;
    std::copy(vals, vals + Synapse::DATA_POINTS, std::begin(setpoints));
    setSetpoints(setpoints);
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderRaw(int j, double * v)
{
    yCTrace(DEXTRA, "%d", j);
    CHECK_JOINT(j);
    *v = getSetpoint(j);
    return true;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncodersRaw(double *encs)
{
    //CD_DEBUG("\n");
    Synapse::Setpoints setpoints;
    getSetpoints(setpoints);
    std::copy(std::cbegin(setpoints), std::cend(setpoints), encs);
    return true;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderSpeedRaw(int j, double * sp)
{
    //CD_DEBUG("(%d)\n", j); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    return false;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderSpeedsRaw(double * spds)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderSpeedRaw(j, &spds[j]);
    }

    return ok;
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderAccelerationRaw(int j, double * accs)
{
    //CD_DEBUG("(%d)\n", j); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    return false;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderAccelerationsRaw(double * accs)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream

    bool ok = true;

    for (int j = 0; j < Synapse::DATA_POINTS; j++)
    {
        ok &= getEncoderAccelerationRaw(j, &accs[j]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool DextraRawControlboard::getEncoderTimedRaw(int j, double * enc, double * time)
{
    //CD_DEBUG("(%d)\n", j); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    *time = yarp::os::Time::now();
    return getEncoderRaw(j, enc);
}

// ------------------------------------------------------------------------------

bool DextraRawControlboard::getEncodersTimedRaw(double * encs, double * time)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    *time = yarp::os::Time::now();
    return getEncodersRaw(encs);
}

// -----------------------------------------------------------------------------
