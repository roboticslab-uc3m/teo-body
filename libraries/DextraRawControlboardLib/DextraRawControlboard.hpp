// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DEXTRA_RAW_CONTROLBOARD_HPP__
#define __DEXTRA_RAW_CONTROLBOARD_HPP__

#include <mutex>

#include <yarp/dev/ControlBoardInterfaces.h>

#include "Synapse.hpp"

#define CHECK_JOINT(j) do { if ((j) < 0 || (j) >= Synapse::DATA_POINTS) return false; } while (0)

namespace roboticslab
{

/**
 * @ingroup yarp_devices_libraries
 * @defgroup DextraRawControlboard
 * @brief Contains roboticslab::DextraRawControlboard.
 */

/**
 * @ingroup DextraRawControlboard
 * @brief Base implementation for the custom UC3M Dextra Hand controlboard interfaces.
 */
class DextraRawControlboard : public yarp::dev::IAxisInfoRaw,
                              public yarp::dev::IControlLimitsRaw,
                              public yarp::dev::IControlModeRaw,
                              public yarp::dev::IEncodersTimedRaw,
                              public yarp::dev::IPositionControlRaw,
                              public yarp::dev::IPositionDirectRaw,
                              public yarp::dev::IVelocityControlRaw
{
public:

    DextraRawControlboard();
    ~DextraRawControlboard() override = default;

    void acquireSynapseHandle(Synapse * synapse);
    void destroySynapse();

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    bool getAxisNameRaw(int axis, std::string & name) override;
    bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override;

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

    bool setLimitsRaw(int axis, double min, double max) override;
    bool getLimitsRaw(int axis, double * min, double * max) override;
    bool setVelLimitsRaw(int axis, double min, double max) override;
    bool getVelLimitsRaw(int axis, double * min, double * max) override;

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override;
    bool getControlModesRaw(int * modes) override;
    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override;
    bool setControlModeRaw(int j, int mode) override;
    bool setControlModesRaw(int * modes) override;
    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getAxes(int * ax) override;
    bool resetEncoderRaw(int j) override;
    bool resetEncodersRaw() override;
    bool setEncoderRaw(int j, double val) override;
    bool setEncodersRaw(const double * vals) override;
    bool getEncoderRaw(int j, double * v) override;
    bool getEncodersRaw(double * encs) override;
    bool getEncoderSpeedRaw(int j, double * sp) override;
    bool getEncoderSpeedsRaw(double * spds) override;
    bool getEncoderAccelerationRaw(int j, double * spds) override;
    bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getEncoderTimedRaw(int j, double * encs, double * time) override;
    bool getEncodersTimedRaw(double * encs, double * time) override;

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    //bool getAxes(int * ax) override;
    bool positionMoveRaw(int j, double ref) override;
    bool positionMoveRaw(const double * refs) override;
    bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override;
    bool relativeMoveRaw(int j, double delta) override;
    bool relativeMoveRaw(const double * deltas) override;
    bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override;
    bool checkMotionDoneRaw(int j, bool * flag) override;
    bool checkMotionDoneRaw(bool * flag) override;
    bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override;
    bool setRefSpeedRaw(int j, double sp) override;
    bool setRefSpeedsRaw(const double * spds) override;
    bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override;
    bool setRefAccelerationRaw(int j, double acc) override;
    bool setRefAccelerationsRaw(const double * accs) override;
    bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    bool getRefSpeedRaw(int j, double * ref) override;
    bool getRefSpeedsRaw(double * spds) override;
    bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override;
    bool getRefAccelerationRaw(int j, double * acc) override;
    bool getRefAccelerationsRaw(double * accs) override;
    bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    bool stopRaw(int j) override;
    bool stopRaw() override;
    bool stopRaw(int n_joint, const int * joints) override;
    bool getTargetPositionRaw(int joint, double * ref) override;
    bool getTargetPositionsRaw(double * refs) override;
    bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override;

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    bool setPositionRaw(int j, double ref) override;
    bool setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    bool setPositionsRaw(const double *refs) override;

    //  --------- IVelocityControlRaw declarations and stub implementations. ---------

    // re-implemented methods have been omitted
    bool velocityMoveRaw(int j, double sp) override { return false; }
    bool velocityMoveRaw(const double * sp) override { return false; }
    bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override { return false; }
    bool getRefVelocityRaw(int joint, double * vel) override { return false; }
    bool getRefVelocitiesRaw(double * vels) override { return false; }
    bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override { return false; }

protected:

    Synapse * synapse;

private:

    double getSetpoint(int j);
    void getSetpoints(Synapse::Setpoints & setpoints);
    void setSetpoint(int j, Synapse::setpoint_t setpoint);
    void setSetpoints(const Synapse::Setpoints & setpoints);

    Synapse::Setpoints setpoints;
    mutable std::mutex setpointMutex;
};

} // namespace roboticslab

#endif // __DEXTRA_RAW_CONTROLBOARD_HPP__
