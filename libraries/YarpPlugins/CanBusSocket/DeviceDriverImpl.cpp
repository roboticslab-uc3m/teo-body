// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusSocket.hpp"

using namespace roboticslab;

// ------------------- DeviceDriver Related ------------------------------------

bool CanBusSocket::open(yarp::os::Searchable& config)
{
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusSocket::close()
{
    return true;
}

// -----------------------------------------------------------------------------
