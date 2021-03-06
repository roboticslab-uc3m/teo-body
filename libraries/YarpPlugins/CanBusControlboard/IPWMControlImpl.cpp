// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefDutyCycle(int m, double ref)
{
    yTrace("%d %f", m, ref);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::setRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefDutyCycles(const double * refs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::setRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefDutyCycle(int m, double * ref)
{
    yTrace("%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getRefDutyCycleRaw, m, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefDutyCycles(double * refs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getRefDutyCyclesRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getDutyCycle(int m, double * val)
{
    yTrace("%d", m);
    CHECK_JOINT(m);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPWMControlRaw::getDutyCycleRaw, m, val);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getDutyCycles(double * vals)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPWMControlRaw::getDutyCyclesRaw, vals);
}

// -----------------------------------------------------------------------------
