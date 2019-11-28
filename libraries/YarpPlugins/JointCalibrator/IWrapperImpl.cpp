// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "JointCalibrator.hpp"

using namespace roboticslab;

bool JointCalibrator::attach(yarp::dev::PolyDriver * poly)
{
    int localAxes;

    return poly->view(iControlMode)
            && poly->view(iEncoders)
            && poly->view(iPositionControl)
            && iEncoders->getAxes(&localAxes)
            && localAxes == axes;
}

bool JointCalibrator::detach()
{
    return true;
}
