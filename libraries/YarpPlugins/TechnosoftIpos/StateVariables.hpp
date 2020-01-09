// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __IPOS_STATE_VARIABLES_HPP__
#define __IPOS_STATE_VARIABLES_HPP__

#include <cstdint>

#include <atomic>
#include <bitset>
#include <memory>
#include <mutex>
#include <string>

#include <yarp/conf/numeric.h>
#include <yarp/os/Stamp.h>

#include "PdoProtocol.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

/**
 * @ingroup TechnosoftIpos
 * @brief Stores last encoder reads, obtains mean speeds and accelerations
 * via differentiation.
 */
class EncoderRead
{
public:
    //! Constructor, set internals to zero.
    EncoderRead();

    //! Constructor, set initial position (internal units).
    EncoderRead(std::int32_t initialPos);

    //! Set new position (internal units) with timestamp, update speeds and accelerations.
    void update(std::int32_t newPos, double newTime = 0.0);

    //! Reset internals to zero, pick provided position (internal units).
    void reset(std::int32_t pos = 0);

    //! Retrieve current position (internal units).
    std::int32_t queryPosition() const;

    //! Retrieve instantaneous speed (internal units).
    double querySpeed() const;

    //! Retrieve instantaneous acceleration (internal units).
    double queryAcceleration() const;

    //! Retrieve last timestamp.
    double queryTime() const;

private:
    std::int32_t lastPosition, nextToLastPosition;
    double lastSpeed, nextToLastSpeed;
    double lastAcceleration;
    yarp::os::Stamp lastStamp;
    mutable std::mutex encoderMutex;
};

/**
 * @ingroup TechnosoftIpos
 * @brief Provides access to internal iPOS variables and helpful conversion methods.
 */
struct StateVariables
{
    //! Make sure stored variables actually make sense.
    bool validateInitialState(unsigned int canId);

    //! Wait with timeout for requested control mode change.
    bool awaitControlMode(yarp::conf::vocab32_t mode);

    //! Convert position, speed or acceleration to internal units.
    double degreesToInternalUnits(double value, int derivativeOrder = 0) const;

    //! Convert position, speed or acceleration to degrees.
    double internalUnitsToDegrees(double value, int derivativeOrder = 0) const;

    //! Convert current to internal units.
    std::int16_t currentToInternalUnits(double value) const;

    //! Convert current to amperes.
    double internalUnitsToCurrent(std::int16_t value) const;

    //! Apply internal iPOS conversion to express drive peak current in amperes.
    double internalUnitsToPeakCurrent(std::int16_t value) const;

    //! Convert current (amperes) to torque (Nm).
    double currentToTorque(double current) const;

    //! Convert torque (Nm) to current (amperes).
    double torqueToCurrent(double torque) const;

    std::unique_ptr<StateObserver> controlModeObserverPtr {new StateObserver(1.0)}; // arbitrary 1 second wait

    // read/write, no concurrent access

    std::bitset<16> msr;
    std::bitset<16> mer;
    std::bitset<16> der;
    std::bitset<16> der2;
    std::bitset<16> cer;
    std::bitset<16> ptStatus;
    std::int8_t modesOfOperation = 0;
    std::uint8_t nmtState = 0;

    // read/write, those require atomic access

    EncoderRead lastEncoderRead;
    std::atomic<std::int16_t> lastCurrentRead;

    std::atomic<yarp::conf::vocab32_t> actualControlMode;
    std::atomic<yarp::conf::vocab32_t> requestedcontrolMode;

    std::atomic<double> tr;
    std::atomic<double> k;
    std::atomic<int> encoderPulses;
    std::atomic<int> pulsesPerSample;

    // read only, conceptually immutable

    double drivePeakCurrent = 0.0;
    double maxVel = 0.0;

    std::string axisName;
    yarp::conf::vocab32_t jointType = 0;

    bool reverse = false;

    PdoConfiguration tpdo1Conf;
    PdoConfiguration tpdo2Conf;
    PdoConfiguration tpdo3Conf;

    // read only, fresh values queried from iPOS drive

    double min = 0.0;
    double max = 0.0;
    double refSpeed = 0.0;
    double refAcceleration = 0.0;
};

} // namespace roboticslab

#endif // __IPOS_STATE_VARIABLES_HPP__
