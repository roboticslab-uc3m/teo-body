// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __MOTOR_IPOS__
#define __MOTOR_IPOS__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

//#define CD_FULL_FILE  //-- Good for debugging as intended for polymorphism.
//#define CD_HIDE_DEBUG
#define CD_HIDE_SUCCESS
#define CD_HIDE_INFO
//#define CD_HIDE_WARNING
//#define CD_HIDE_ERROR
#include "ColorDebug.hpp"
#include "../ICanBusSharer.h"

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 *
 * @ingroup bodybot
 * @brief Specifies the Technosoft iPOS behaviour and specifications.
 *
 */
// Note: IEncodersTimedRaw inherits from IEncodersRaw
class MotorIpos : public DeviceDriver, public IControlLimitsRaw, public IControlModeRaw, public IEncodersTimedRaw,
        public IPositionControlRaw, public IPositionDirectRaw, public IVelocityControlRaw, public ITorqueControlRaw,
        public ICanBusSharer {

    public:

        MotorIpos() {
            //canDevicePtr = NULL;
        }

        //  --------- DeviceDriver Declarations. Implementation in MotorIpos.cpp ---------
        virtual bool open(Searchable& config);
        virtual bool close();

        //  --------- ICanBusSharer Declarations. Implementation in MotorIpos.cpp ---------
        virtual bool setCanBusPtr(CanBusHico *canDevicePtr);        
        virtual bool interpretMessage( can_msg * message);
        /** "start". Figure 5.1 Drive’s status machine. States and transitions (p68, 84/263). */
        virtual bool start();
        /** "ready to switch on", also acts as "shutdown" */
        virtual bool readyToSwitchOn();
        /** "switch on", also acts as "disable operation" */
        virtual bool switchOn();
        /** enable */
        virtual bool enable();

        //  --------- IControlLimitsRaw Declarations. Implementation in IControlLimitsRawImpl.cpp ---------
        virtual bool setLimitsRaw(int axis, double min, double max);
        virtual bool getLimitsRaw(int axis, double *min, double *max);

        //  --------- IControlModeRaw Declarations. Implementation in IControlModeRawImpl.cpp ---------
        virtual bool setPositionModeRaw(int j);
        virtual bool setVelocityModeRaw(int j);
        virtual bool setTorqueModeRaw(int j);
        virtual bool setImpedancePositionModeRaw(int j);
        virtual bool setImpedanceVelocityModeRaw(int j);
        virtual bool setOpenLoopModeRaw(int j);
        virtual bool getControlModeRaw(int j, int *mode);
        virtual bool getControlModesRaw(int *modes){
            CD_ERROR("\n");
            return false;
        }

        //  ---------- IEncodersRaw Declarations. Implementation in IEncodersRawImpl.cpp ----------
        virtual bool resetEncoderRaw(int j);
        virtual bool resetEncodersRaw() {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setEncoderRaw(int j, double val);
        virtual bool setEncodersRaw(const double *vals) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderRaw(int j, double *v);
        virtual bool getEncodersRaw(double *encs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderSpeedRaw(int j, double *sp);
        virtual bool getEncoderSpeedsRaw(double *spds) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderAccelerationRaw(int j, double *spds);
        virtual bool getEncoderAccelerationsRaw(double *accs) {
            CD_ERROR("\n");
            return false;
        }

        //  ---------- IEncodersTimedRaw Declarations. Implementation in IEncodersTimedRawImpl.cpp ----------
        virtual bool getEncodersTimedRaw(double *encs, double *time) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getEncoderTimedRaw(int j, double *encs, double *time);

        // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------
        virtual bool getAxes(int *ax){
            *ax = 1;
            return true;
        }
        virtual bool setPositionModeRaw() {
            CD_ERROR("\n");
            return false;
        }
        virtual bool positionMoveRaw(int j, double ref);
        virtual bool positionMoveRaw(const double *refs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool relativeMoveRaw(int j, double delta);
        virtual bool relativeMoveRaw(const double *deltas) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool checkMotionDoneRaw(int j, bool *flag);
        virtual bool checkMotionDoneRaw(bool *flag) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setRefSpeedRaw(int j, double sp);
        virtual bool setRefSpeedsRaw(const double *spds) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setRefAccelerationRaw(int j, double acc);
        virtual bool setRefAccelerationsRaw(const double *accs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getRefSpeedRaw(int j, double *ref);
        virtual bool getRefSpeedsRaw(double *spds) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getRefAccelerationRaw(int j, double *acc);
        virtual bool getRefAccelerationsRaw(double *accs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool stopRaw(int j);
        virtual bool stopRaw() {
            CD_ERROR("\n");
            return false;
        }

        // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------
        virtual bool setPositionDirectModeRaw();
        virtual bool setPositionRaw(int j, double ref);
        virtual bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
        virtual bool setPositionsRaw(const double *refs);

        // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------
        virtual bool setTorqueModeRaw() {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getRefTorquesRaw(double *t) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getRefTorqueRaw(int j, double *t);
        virtual bool setRefTorquesRaw(const double *t) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setRefTorqueRaw(int j, double t);
        virtual bool getBemfParamRaw(int j, double *bemf);
        virtual bool setBemfParamRaw(int j, double bemf);
        virtual bool setTorquePidRaw(int j, const Pid &pid);
        virtual bool getTorqueRaw(int j, double *t);
        virtual bool getTorquesRaw(double *t) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getTorqueRangeRaw(int j, double *min, double *max);
        virtual bool getTorqueRangesRaw(double *min, double *max) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setTorquePidsRaw(const Pid *pids) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool setTorqueErrorLimitRaw(int j, double limit);
        virtual bool setTorqueErrorLimitsRaw(const double *limits) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getTorqueErrorRaw(int j, double *err);
        virtual bool getTorqueErrorsRaw(double *errs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getTorquePidOutputRaw(int j, double *out);
        virtual bool getTorquePidOutputsRaw(double *outs) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getTorquePidRaw(int j, Pid *pid);
        virtual bool getTorquePidsRaw(Pid *pids) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool getTorqueErrorLimitRaw(int j, double *limit);
        virtual bool getTorqueErrorLimitsRaw(double *limits) {
            CD_ERROR("\n");
            return false;
        }
        virtual bool resetTorquePidRaw(int j);
        virtual bool disableTorquePidRaw(int j);
        virtual bool enableTorquePidRaw(int j);
        virtual bool setTorqueOffsetRaw(int j, double v);

        //  --------- IVelocityControl Declarations. Implementation in IVelocityControlImpl.cpp ---------
        virtual bool setVelocityModeRaw() {
            CD_ERROR("\n");
            return false;
        }
        virtual bool velocityMoveRaw(int j, double sp);
        virtual bool velocityMoveRaw(const double *sp) {
            CD_ERROR("\n");
            return false;
        }

    protected:

        //  --------- Implementation in MotorIpos.cpp ---------

        //-- CAN bus stuff
        /**
         * Write message to the CAN buffer.
         * @param cob Message's COB
         * @param len Data field length
         * @param msgData Data to send
         * @return true/false on success/failure.
         */
        bool send(uint32_t cob, uint16_t len, uint8_t * msgData);

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message);
        std::string msgToStr(uint32_t cob, uint16_t len, uint8_t * msgData);

        int canId;
        CanBusHico *canDevicePtr;
        double lastUsage;

        //-- Encoder stuff
        double encoder;
        uint32_t encoderTimestamp;
        yarp::os::Semaphore encoderReady;

        //-- Mode stuff
        int getMode;
        yarp::os::Semaphore getModeReady;
        bool targetReached;
        yarp::os::Semaphore targetReachedReady;

        //-- PT stuff
        int16_t ptModeMs;  //-- [ms]
        int ptPointCounter;
        bool ptMovementDone;
        yarp::os::Semaphore ptBuffer;

        //-- More internal parameter stuff
        double max, min, refAcceleration, refSpeed, tr;

};

}  // namespace teo

#endif  // __MOTOR_IPOS__

