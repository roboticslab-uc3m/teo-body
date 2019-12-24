// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JOINT_CALIBRATOR_HPP__
#define __JOINT_CALIBRATOR_HPP__

#include <vector>

#include <yarp/dev/CalibratorInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/Wrapper.h>

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup JointCalibrator
 * @brief Contains roboticslab::JointCalibrator.
 */

/**
 * @ingroup JointCalibrator
 * @brief ...
 */
struct MovementSpecs
{
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> acc;
};

/**
 * @ingroup JointCalibrator
 * @brief ...
 */
class JointCalibrator : public yarp::dev::DeviceDriver,
                        public yarp::dev::IRemoteCalibrator,
                        public yarp::dev::IWrapper
{
public:
    JointCalibrator()
        : axes(0), iControlMode(nullptr), iEncoders(nullptr), iPositionControl(nullptr)
    { }

    virtual bool calibrateSingleJoint(int j);
    virtual bool calibrateWholePart();
    virtual bool homingSingleJoint(int j);
    virtual bool homingWholePart();
    virtual bool parkSingleJoint(int j, bool wait);
    virtual bool parkWholePart();
    virtual bool quitCalibrate();
    virtual bool quitPark();

    virtual bool attach(yarp::dev::PolyDriver * poly);
    virtual bool detach();

    virtual bool open(yarp::os::Searchable & config);
    virtual bool close();

private:
    bool move(const std::vector<int> & joints, const MovementSpecs & specs);

    int axes;

    MovementSpecs homeSpecs;
    MovementSpecs parkSpecs;

    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionControl * iPositionControl;
};

}  // namespace roboticslab

#endif // __JOINT_CALIBRATOR_HPP__