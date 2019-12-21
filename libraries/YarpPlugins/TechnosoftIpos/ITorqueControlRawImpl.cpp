// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getRefTorqueRaw(int j, double * t)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);

    return can->sdo()->upload<std::int32_t>("External online reference", [&](std::int32_t data)
            { double curr = vars.internalUnitsToCurrent(data >> 16);
              *t = vars.currentToTorque(curr); },
            0x201C);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getRefTorquesRaw(double * t)
{
    CD_DEBUG("\n");
    return getRefTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setRefTorqueRaw(int j, double t)
{
    CD_DEBUG("(%d, %f)\n", j, t);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);

    double curr = vars.torqueToCurrent(t);
    std::int32_t data = vars.currentToInternalUnits(curr) << 16;
    return quitHaltState(VOCAB_CM_TORQUE) && can->sdo()->download("External online reference", data, 0x201C);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setRefTorquesRaw(const double * t)
{
    CD_DEBUG("\n");
    return setRefTorqueRaw(0, t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRaw(int j, double * t)
{
    //CD_DEBUG("(%d)\n", j); // too verbose in controlboardwrapper2 stream
    CHECK_JOINT(j);
    std::int16_t temp = vars.lastCurrentRead;
    double curr = vars.internalUnitsToCurrent(temp);
    *t = vars.currentToTorque(curr);
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorquesRaw(double * t)
{
    //CD_DEBUG("\n"); // too verbose in controlboardwrapper2 stream
    return getTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRangeRaw(int j, double * min, double * max)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint16_t>("Current limit", [&](std::uint16_t data)
            { double temp = vars.internalUnitsToPeakCurrent(data);
              *max = vars.currentToTorque(temp);
              *min = -(*max); },
            0x207F);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRangesRaw(double * min, double * max)
{
    CD_DEBUG("\n");
    return getTorqueRangeRaw(0, &min[0], &max[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = vars.k;
    params->ktau_scale = 0.0;

    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    vars.k = params.ktau;
    return true;
}

// -------------------------------------------------------------------------------------
