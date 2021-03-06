// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusPeak.hpp"

#include <cstring> // std::strerror

#include <string>

#include <yarp/os/LogStream.h>

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::CanBusPeak::open(yarp::os::Searchable& config)
{
    yDebug() << "CanBusPeak config:" << config.toString();

    std::string devicePath = config.check("port", yarp::os::Value(DEFAULT_PORT), "CAN device path").asString();

    int bitrate = config.check("bitrate", yarp::os::Value(DEFAULT_BITRATE), "CAN bitrate (bps)").asInt32();

    blockingMode = config.check("blockingMode", yarp::os::Value(DEFAULT_BLOCKING_MODE), "blocking mode enabled").asBool();
    allowPermissive = config.check("allowPermissive", yarp::os::Value(DEFAULT_ALLOW_PERMISSIVE), "read/write permissive mode").asBool();

    int flags = OFD_BITRATE | PCANFD_INIT_STD_MSG_ONLY;

    if (blockingMode)
    {
        yInfo() << "Blocking mode enabled for CAN device" << devicePath;

        rxTimeoutMs = config.check("rxTimeoutMs", yarp::os::Value(DEFAULT_RX_TIMEOUT_MS), "RX timeout (milliseconds)").asInt32();
        txTimeoutMs = config.check("txTimeoutMs", yarp::os::Value(DEFAULT_TX_TIMEOUT_MS), "TX timeout (milliseconds)").asInt32();

        if (rxTimeoutMs <= 0)
        {
            yWarning() << "RX timeout value <= 0, CAN read calls will block until the buffer is ready";
        }

        if (txTimeoutMs <= 0)
        {
            yWarning() << "TX timeout value <= 0, CAN write calls will block until the buffer is ready";
        }
    }
    else
    {
        yInfo() << "Requested non-blocking mode for CAN device" << devicePath;
        flags |= OFD_NONBLOCKING;
    }

    yInfo() << "Permissive mode flag for read/write operations on CAN device" << devicePath << "set to" << allowPermissive;

    int res = pcanfd_open(devicePath.c_str(), flags, bitrate);

    if (res < 0)
    {
        yError("Unable to open CAN device %s (%s)", devicePath.c_str(), std::strerror(-res));
        return false;
    }
    else
    {
        yInfo() << "Successfully opened CAN device" << devicePath;
        fileDescriptor = res;
    }

    if (!config.check("preserveFilters", "don't clear acceptance filters on init"))
    {
        res = pcanfd_del_filters(fileDescriptor);

        if (res < 0)
        {
            yError("Unable to clear acceptance filters on CAN device %s (%s)", devicePath.c_str(), std::strerror(-res));
            return false;
        }
        else
        {
            yInfo() << "Acceptance filters cleared on CAN device" << devicePath;
        }
    }
    else
    {
        yWarning() << "Preserving previous acceptance filters (if any) on" << devicePath;
    }

    //-- Load initial node IDs and set acceptance filters.
    if (config.check("ids", "initial node IDs"))
    {
        const yarp::os::Bottle & ids = config.findGroup("ids").tail();

        if (ids.size() != 0)
        {
            yInfo() << "Parsing bottle of ids on CAN device" << ids.toString();

            for (int i = 0; i < ids.size(); i++)
            {
                activeFilters.insert(ids.get(i).asFloat64());
            }

            std::uint64_t acc = computeAcceptanceCodeAndMask();

            yDebug("New acceptance code+mask: %016lxh", acc);

            res = pcanfd_set_option(fileDescriptor, PCANFD_OPT_ACC_FILTER_11B, &acc, sizeof(acc));

            if (res < 0)
            {
                yError("Unable to set acceptance filters on CAN device: %s (%s)", devicePath.c_str(), std::strerror(-res));
                activeFilters.clear();
                return false;
            }

            yInfo() << "Initial IDs added to set of acceptance filters in CAN device" << devicePath;
        }
        else
        {
            yInfo() << "No bottle of ids given to CAN device" << devicePath;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CanBusPeak::close()
{
    if (fileDescriptor > 0)
    {
        if (!activeFilters.empty())
        {
            int res = pcanfd_del_filters(fileDescriptor);

            if (res < 0)
            {
                yWarning() << "Unable to clear acceptance filters:" << std::strerror(-res);
            }
            else
            {
                activeFilters.clear();
            }
        }

        pcanfd_close(fileDescriptor);
        fileDescriptor = 0;
    }

    return true;
}

// -----------------------------------------------------------------------------
