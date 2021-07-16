// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

// ------------------- IPositionControl related --------------------------------

bool roboticslab::AmorControlboard::getAxes(int *ax)
{
    *ax = AMOR_NUM_JOINTS;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(int j, double ref)
{
    yCTrace(AMOR, "%d %f", j, ref);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    positions[j] = toRad(ref);

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(const double *refs)
{
    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] = toRad(refs[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(int j, double delta)
{
    yCTrace(AMOR, "%d %f", j, delta);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    positions[j] += toRad(delta);

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const double *deltas)
{
    AMOR_VECTOR7 positions;

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        positions[j] += toRad(deltas[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(int j, bool *flag)
{
    if (!indexWithinRange(j))
    {
        return false;
    }

    return checkMotionDone(flag);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(bool *flag)
{
    yCTrace(AMOR, "");

    amor_movement_status status;

    if (amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_movement_status(): %s", amor_error());
        return false;
    }

    *flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeed(int j, double sp)
{
    yCError(AMOR, "setRefSpeed() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const double *spds)
{
    yCError(AMOR, "setRefSpeeds() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAcceleration(int j, double acc)
{
    yCError(AMOR, "setRefAcceleration() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const double *accs)
{
    yCError(AMOR, "setRefAccelerations() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeed(int j, double *ref)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
        return false;
    }

    *ref = toDeg(parameters.maxVelocity);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(double *spds)
{
    yCTrace(AMOR, "");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAcceleration(int j, double *acc)
{
    yCTrace(AMOR, "%d", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    AMOR_JOINT_INFO parameters;

    if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
        return false;
    }

    *acc = toDeg(parameters.maxAcceleration);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(double *accs)
{
    yCTrace(AMOR, "");

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, j, &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(int j)
{
    yCWarning(AMOR, "Selective stop not available, stopping all joints at once (%d)", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop()
{
    yCTrace(AMOR, "");
    return amor_controlled_stop(handle) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::positionMove(const int n_joint, const int *joints, const double *refs)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] = toRad(refs[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (n_joint < AMOR_NUM_JOINTS && amor_get_actual_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_actual_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        positions[joints[j]] += toRad(deltas[j]);
    }

    return amor_set_positions(handle, positions) == AMOR_SUCCESS;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    amor_movement_status status;

    if (amor_get_movement_status(handle, &status) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_movement_status(): %s", amor_error());
        return false;
    }

    bool flag = (status == AMOR_MOVEMENT_STATUS_FINISHED);

    for (int j = 0; j < n_joint; j++)
    {
        flags[j] = flag;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefSpeeds(const int n_joint, const int *joints, const double *spds)
{
    yCError(AMOR, "setRefSpeeds() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    yCError(AMOR, "setRefAccelerations() not available");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        spds[j] = toDeg(parameters.maxVelocity);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        AMOR_JOINT_INFO parameters;

        if (amor_get_joint_info(handle, joints[j], &parameters) != AMOR_SUCCESS)
        {
            yCError(AMOR, "amor_get_joint_info(): %s", amor_error());
            return false;
        }

        accs[j] = toDeg(parameters.maxAcceleration);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::stop(const int n_joint, const int *joints)
{
    yCWarning(AMOR, "Selective stop not available, stopping all joints at once (%d)", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    return stop();
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPosition(const int joint, double *ref)
{
    yCTrace(AMOR, "%d", joint);

    if (!indexWithinRange(joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_positions(): %s", amor_error());
        return false;
    }

    *ref = toDeg(positions[joint]);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPositions(double *refs)
{
    yCTrace(AMOR, "");

    AMOR_VECTOR7 positions;

    if (amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < AMOR_NUM_JOINTS; j++)
    {
        refs[j] = toDeg(positions[j]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    yCTrace(AMOR, "%d", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    AMOR_VECTOR7 positions;

    if (amor_get_req_positions(handle, &positions) != AMOR_SUCCESS)
    {
        yCError(AMOR, "amor_get_req_positions(): %s", amor_error());
        return false;
    }

    for (int j = 0; j < n_joint; j++)
    {
        refs[j] = toDeg(positions[joints[j]]);
    }

    return true;
}

// -----------------------------------------------------------------------------
