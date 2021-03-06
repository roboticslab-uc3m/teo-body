// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <cstring> // std::strerror
#include <cerrno>

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusHico::open(yarp::os::Searchable& config)
{
    yDebug() << "CanBusHico config:" << config.toString();

    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();
    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "CAN blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "CAN read/write permissive mode").asBool();

    if (blockingMode)
    {
        yInfo() << "Blocking mode enabled for CAN device" << devicePath;

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "CAN RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "CAN TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yWarning() << "RX timeout value <= 0, CAN read calls will block until the buffer is ready:" << devicePath;
        }

        if (txTimeoutMs <= 0)
        {
            yWarning() << "TX timeout value <= 0, CAN write calls will block until the buffer is ready:" << devicePath;
        }
    }
    else
    {
        yInfo() << "Requested non-blocking mode for CAN device" << devicePath;
    }

    yInfo("Permissive mode flag for read/write operations on CAN device %s: %d", devicePath.c_str(), allowPermissive);

    std::string filterConfigStr = config.check("filterConfiguration", yarp::os::Value(DEFAULT_FILTER_CONFIGURATION),
            "CAN filter configuration (disabled|noRange|maskAndRange)").asString();

    yInfo("CAN filter configuration for CAN device %s: %s", devicePath.c_str(), filterConfigStr.c_str());

    filterConfig = FilterManager::parseFilterConfiguration(filterConfigStr);

    //-- Open the CAN device for reading and writing.
    fileDescriptor = ::open(devicePath.c_str(), O_RDWR);

    if (fileDescriptor == -1)
    {
        yError() << "Could not open CAN device" << devicePath;
        return false;
    }

    yInfo() << "Opened CAN device" << devicePath;

    initBitrateMap();

    //-- Set the CAN bitrate.
    if (!canSetBaudRate(bitrate))
    {
        yError() << "Could not set bitrate on CAN device" << devicePath;
        return false;
    }

    yInfo() << "Bitrate set on CAN device" << devicePath;

    if (!blockingMode)
    {
        int fcntlFlags = ::fcntl(fileDescriptor, F_GETFL);

        if (fcntlFlags == -1)
        {
            yError() << "Unable to retrieve FD flags on CAN device" << devicePath;
            return false;
        }

        fcntlFlags |= O_NONBLOCK;

        if (::fcntl(fileDescriptor, F_SETFL, fcntlFlags) == -1)
        {
            yError("Unable to set non-blocking mode on CAN device %s; fcntl() error: %s", devicePath.c_str(), std::strerror(errno));
            return false;
        }

        yInfo() << "Non-blocking mode enabled on CAN device" << devicePath;
    }

    if (filterConfig != FilterManager::DISABLED)
    {
        filterManager = new FilterManager(fileDescriptor, filterConfig == FilterManager::MASK_AND_RANGE);

        if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
        {
            if (!filterManager->clearFilters())
            {
                yError() << "Unable to clear acceptance filters on CAN device" << devicePath;
                return false;
            }
            else
            {
                yInfo() << "Acceptance filters cleared on CAN device" << devicePath;
            }
        }
        else
        {
            yWarning() << "Preserving previous acceptance filters (if any) on CAN device" << devicePath;
        }

        //-- Load initial node IDs and set acceptance filters.
        if (config.check("ids", "initial node IDs"))
        {
            const yarp::os::Bottle & ids = config.findGroup("ids").tail();

            if (ids.size() != 0)
            {
                yInfo() << "Parsing bottle of ids on CAN device" << ids.toString();

                if (!filterManager->parseIds(ids))
                {
                    yError() << "Could not set acceptance filters on CAN device" << devicePath;
                    return false;
                }

                if (!filterManager->isValid())
                {
                    yWarning() << "Hardware limit was hit on CAN device" << devicePath << "and no acceptance filters are enabled";
                }
            }
            else
            {
                yInfo() << "No bottle of ids given to CAN device" << devicePath;
            }
        }
    }
    else
    {
        yInfo() << "Acceptance filters are disabled for CAN device" << devicePath;
    }

    //-- Start the CAN device.
    if (::ioctl(fileDescriptor,IOC_START) == -1)
    {
        yError() << "IOC_START failed on CAN device" << devicePath;
        return false;
    }

    yInfo() << "IOC_START ok on CAN device" << devicePath;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusHico::close()
{
    if (fileDescriptor > 0)
    {
        if (filterConfig != FilterManager::DISABLED && filterManager != NULL)
        {
            filterManager->clearFilters();
            delete filterManager;
            filterManager = NULL;
        }

        ::close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
