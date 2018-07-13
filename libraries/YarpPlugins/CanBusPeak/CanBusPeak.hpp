// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_PEAK__
#define __CAN_BUS_PEAK__

#include <cstdint>

#include <set>

#include <yarp/os/Semaphore.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include <libpcanfd.h>

#include "PeakCanMessage.hpp"

#define DEFAULT_CAN_DEVICE "/dev/pcan0"
#define DEFAULT_CAN_BITRATE 1000000

#define DEFAULT_CAN_RX_TIMEOUT_MS 1
#define DEFAULT_CAN_TX_TIMEOUT_MS 0  // '0' means no timeout

namespace roboticslab
{

/**
 *
 * @ingroup CanBusPeak
 * @brief Specifies the PeakCan behaviour and specifications.
 *
 */
class CanBusPeak : public yarp::dev::DeviceDriver,
                   public yarp::dev::ICanBus,
                   public yarp::dev::ImplementCanBufferFactory<PeakCanMessage, struct pcanfd_msg>
{

public:

    CanBusPeak() : fileDescriptor(0),
                   rxTimeoutMs(DEFAULT_CAN_RX_TIMEOUT_MS),
                   txTimeoutMs(DEFAULT_CAN_TX_TIMEOUT_MS),
                   nonBlockingMode(false)
    {}

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    /** Initialize the CAN device.
     * @param device is the device path, such as "/dev/can0".
     * @param bitrate is the bitrate, such as BITRATE_100k.
     * @return true/false on success/failure.
     */
    virtual bool open(yarp::os::Searchable& config);

    /** Close the CAN device. */
    virtual bool close();

    //  --------- ICanBus declarations. Implementation in ICanBusImpl.cpp ---------

    virtual bool canSetBaudRate(unsigned int rate);

    virtual bool canGetBaudRate(unsigned int * rate);

    virtual bool canIdAdd(unsigned int id);

    virtual bool canIdDelete(unsigned int id);

    virtual bool canRead(yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * read, bool wait = false);

    virtual bool canWrite(const yarp::dev::CanBuffer & msgs, unsigned int size, unsigned int * sent, bool wait = false);

protected:

    enum io_operation { READ, WRITE };

    bool waitUntilTimeout(io_operation op, bool * bufferReady);

    uint64_t computeAcceptanceCodeAndMask();

    int fileDescriptor;
    int rxTimeoutMs;
    int txTimeoutMs;

    bool nonBlockingMode;

    mutable yarp::os::Semaphore canBusReady;

    std::set<unsigned int> activeFilters;
};

}  // namespace roboticslab

#endif  // __CAN_BUS_PEAK__
