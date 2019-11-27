// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <map>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    const std::string slash = yarp::os::NetworkBase::getPathSeparator();

    if (!config.check("bus", "CAN bus") || !config.check("nodes", "CAN nodes"))
    {
        CD_ERROR("Some mandatory keys are missing: \"bus\", \"nodes\".\n");
        return false;
    }

    std::string bus = config.find("bus").asString();
    yarp::os::Bottle * nodes = config.find("nodes").asList();

    if (bus.empty() || nodes == nullptr)
    {
        CD_ERROR("Illegal key(s): empty \"bus\" or nullptr \"nodes\".\n");
        return false;
    }

    std::string busPath = rf.findFileByName("buses" + slash + bus + ".ini");

    yarp::os::Property canBusOptions;
    canBusOptions.setMonitor(config.getMonitor(), bus.c_str());

    if (!canBusOptions.fromConfigFile(busPath))
    {
        CD_ERROR("File %s does not exist or unsufficient permissions.\n", busPath.c_str());
        return false;
    }

    canBusOptions.put("canBlockingMode", false); // enforce non-blocking mode
    canBusOptions.put("canAllowPermissive", false); // always check usage requirements

    if (!canBusDevice.open(canBusOptions))
    {
        CD_ERROR("canBusDevice instantiation not worked.\n");
        return false;
    }

    if (!canBusDevice.view(iCanBus))
    {
        CD_ERROR("Cannot view ICanBus interface in device: %s.\n", bus.c_str());
        return false;
    }

    yarp::dev::ICanBufferFactory * iCanBufferFactory;

    if (!canBusDevice.view(iCanBufferFactory))
    {
        CD_ERROR("Cannot view ICanBufferFactory interface in device: %s.\n", bus.c_str());
        return false;
    }

    iCanBusSharers.resize(nodes->size());

    int cuiTimeout  = config.check("waitEncoder", yarp::os::Value(DEFAULT_CUI_TIMEOUT), "CUI timeout (seconds)").asInt32();

    std::string canBusType = config.check("canBusType", yarp::os::Value(DEFAULT_CAN_BUS), "CAN bus device name").asString();
    int canRxBufferSize = config.check("canBusRxBufferSize", yarp::os::Value(DEFAULT_CAN_RX_BUFFER_SIZE), "CAN bus RX buffer size").asInt();
    int canTxBufferSize = config.check("canBusTxBufferSize", yarp::os::Value(DEFAULT_CAN_TX_BUFFER_SIZE), "CAN bus TX buffer size").asInt();
    double canRxPeriodMs = config.check("canRxPeriodMs", yarp::os::Value(DEFAULT_CAN_RX_PERIOD_MS), "CAN bus RX period (milliseconds)").asFloat64();
    double canTxPeriodMs = config.check("canTxPeriodMs", yarp::os::Value(DEFAULT_CAN_TX_PERIOD_MS), "CAN bus TX period (milliseconds)").asFloat64();
    double canSdoTimeoutMs = config.check("canSdoTimeoutMs", yarp::os::Value(DEFAULT_CAN_SDO_TIMEOUT_MS), "CAN bus SDO timeout (milliseconds)").asFloat64();
    double canDriveStateTimeout = config.check("canDriveStateTimeout", yarp::os::Value(DEFAULT_CAN_DRIVE_STATE_TIMEOUT), "CAN drive state timeout (seconds)").asFloat64();

    linInterpPeriodMs = config.check("linInterpPeriodMs", yarp::os::Value(DEFAULT_LIN_INTERP_PERIOD_MS), "linear interpolation mode period (milliseconds)").asInt32();
    linInterpBufferSize = config.check("linInterpBufferSize", yarp::os::Value(DEFAULT_LIN_INTERP_BUFFER_SIZE), "linear interpolation mode buffer size").asInt32();
    linInterpMode = config.check("linInterpMode", yarp::os::Value(DEFAULT_LIN_INTERP_MODE), "linear interpolation mode (pt/pvt)").asString();

    int parallelCanThreadLimit = config.check("parallelCanThreadLimit", yarp::os::Value(0), "parallel CAN TX thread limit").asInt32();

    if (parallelCanThreadLimit > 0)
    {
        deviceMapper.enableParallelization(parallelCanThreadLimit);
    }

    for (int i = 0; i < nodes->size(); i++)
    {
        std::string node = nodes->get(i).asString();
        std::string nodePath = rf.findFileByName("nodes" + slash + node + ".ini");

        yarp::os::Property nodeOptions;
        nodeOptions.setMonitor(config.getMonitor(), node.c_str());

        if (!nodeOptions.fromConfigFile(nodePath))
        {
            CD_ERROR("File %s does not exist or unsufficient permissions.\n", nodePath.c_str());
            return false;
        }

        yarp::dev::PolyDriver * device = new yarp::dev::PolyDriver;
        nodeDevices.push(device, node.c_str());

        if (!device->open(nodeOptions))
        {
            CD_ERROR("CAN node device %s configuration failure.\n", node.c_str());
            return false;
        }

        if (!deviceMapper.registerDevice(device))
        {
            CD_ERROR("Unable to register device.\n");
            return false;
        }

        if (!device->view(iCanBusSharers[i]))
        {
            CD_ERROR("Unable to view ICanBusSharer.\n");
            return false;
        }

        iCanBusSharers[i]->registerSender(canWriterThread->getDelegate());

        if (!iCanBus->canIdAdd(iCanBusSharers[i]->getId()))
        {
            CD_ERROR("Cannot register acceptance filter for node ID: %d.\n", iCanBusSharers[i]->getId());
            return false;
        }
    }

    const std::string canDevice = canBusOptions.find("canDevice").asString();

    canReaderThread = new CanReaderThread(canDevice, iCanBusSharers);
    canReaderThread->setCanHandles(iCanBus, iCanBufferFactory, canRxBufferSize);
    canReaderThread->setPeriod(canRxPeriodMs);
    canReaderThread->start();

    canWriterThread = new CanWriterThread(canDevice);
    canWriterThread->setCanHandles(iCanBus, iCanBufferFactory, canTxBufferSize);
    canWriterThread->setPeriod(canTxPeriodMs);
    canWriterThread->start();

    for (auto p : iCanBusSharers)
    {
        if (!p->initialize())
        {
            return false;
        }
    }

    posdThread = new PositionDirectThread(deviceMapper, linInterpPeriodMs * 0.001);
    posdThread->start();

    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::close()
{
    bool ok = true;

    if (posdThread && posdThread->isRunning())
    {
        posdThread->stop();
    }

    delete posdThread;

    for (auto p : iCanBusSharers)
    {
        ok &= p->finalize();
    }

    for (int i = 0; i < nodeDevices.size(); i++)
    {
        ok &= nodeDevices[i]->poly->close();
        delete nodeDevices[i]->poly;
    }

    if (canWriterThread && canWriterThread->isRunning())
    {
        ok &= canWriterThread->stop();
    }

    delete canWriterThread;

    if (canReaderThread && canReaderThread->isRunning())
    {
        ok &= canReaderThread->stop();
    }

    delete canReaderThread;

    // Clear CAN acceptance filters ('0' = all IDs that were previously set by canIdAdd).
    if (!iCanBus->canIdDelete(0))
    {
        CD_WARNING("CAN filters may be preserved on the next run.\n");
    }

    ok &= canBusDevice.close();
    return ok;
}

// -----------------------------------------------------------------------------
