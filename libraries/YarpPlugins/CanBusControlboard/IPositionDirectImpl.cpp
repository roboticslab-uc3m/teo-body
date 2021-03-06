// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPosition(int j, double ref)
{
    yTrace("%d %f", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::setPositionRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPositions(const double * refs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::setPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPositions(int n_joint, const int * joints, const double * refs)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::setPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefPosition(int joint, double * ref)
{
    yTrace("%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefPositions(double * refs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefPositions(int n_joint, const int * joints, double * refs)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
