// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

#include <ios>
#include <iomanip>
#include <iostream>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>

YARP_LOG_COMPONENT(DCB, "rl.DumpCanBus")

using namespace roboticslab;

bool DumpCanBus::configure(yarp::os::ResourceFinder & rf)
{
    yCDebug(DCB) << "Config:" << rf.toString();

    if (!rf.check("remote", "remote port name"))
    {
        yCError(DCB) << "Missing remote port name";
        return false;
    }

    std::string local = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT), "local port name").asString();
    std::string remote = rf.find("remote").asString();
    useCanOpen = !rf.check("no-can-open");
    printTimestamp = rf.check("with-ts");

    if (!port.open(local + "/dump:i"))
    {
        yCError(DCB) << "Unable to open local port";
        return false;
    }

    port.setReadOnly();

    yarp::os::ContactStyle style;
    style.carrier = "fast_tcp";
    style.expectReply = false;
    style.persistent = true;
    style.persistenceType = yarp::os::ContactStyle::END_WITH_TO_PORT;

    if (!yarp::os::Network::connect(remote + "/dump:o", port.getName(), style))
    {
        yCError(DCB) << "Unable to connect to remote port";
        return false;
    }

    portReader.attach(port);
    portReader.useCallback(*this);
    portReader.setStrict();

    return true;
}

bool DumpCanBus::close()
{
    portReader.interrupt();
    portReader.disableCallback();
    port.close();
    return true;
}

void DumpCanBus::onRead(yarp::os::Bottle & b)
{
    yarp::os::Stamp lastStamp;
    port.getEnvelope(lastStamp);

    for (auto i = 0; i < b.size(); i++)
    {
        auto * msg = b.get(i).asList();

        if (msg)
        {
            printMessage(*msg, lastStamp);
        }
    }
}

void DumpCanBus::printMessage(const yarp::os::Bottle & b, const yarp::os::Stamp & stamp)
{
    if (printTimestamp)
    {
        std::cout << "[";
        std::cout << std::fixed;
        std::cout << std::setprecision(6);
        std::cout << stamp.getTime();
        std::cout << "] ";
    }

    unsigned int cobId = b.get(0).asInt16();

    std::cout << std::setfill(' ');

    if (!useCanOpen)
    {
        std::cout << std::setw(3) << std::hex << cobId;
    }
    else
    {
        unsigned int id = cobId & 0x7F;

        if (id == 0)
        {
            std::cout << std::setw(10);

            switch (cobId)
            {
            case 0x00:
                std::cout << "NMT";
                break;
            case 0x80:
                std::cout << "SYNC";
                break;
            case 0x100:
                std::cout << "TIME";
                break;
            default:
                std::cout << "UNK!";
                break;
            }
        }
        else
        {
            std::cout << std::setw(3) << std::dec << id;
            std::cout << std::setw(7);

            switch (cobId - id)
            {
            case 0x80:
                std::cout << "EMCY";
                break;
            case 0x180:
                std::cout << "TPDO1";
                break;
            case 0x200:
                std::cout << "RPDO1";
                break;
            case 0x280:
                std::cout << "TPDO2";
                break;
            case 0x300:
                std::cout << "RPDO2";
                break;
            case 0x380:
                std::cout << "TPDO3";
                break;
            case 0x400:
                std::cout << "RPDO3";
                break;
            case 0x480:
                std::cout << "TPDO4";
                break;
            case 0x500:
                std::cout << "RPDO4";
                break;
            case 0x580:
                std::cout << "SDO-U";
                break;
            case 0x600:
                std::cout << "SDO-D";
                break;
            case 0x700:
                std::cout << "HRTBT";
                break;
            default:
                std::cout << "UNK!";
                break;
            }
        }
    }

    if (b.size() > 1)
    {
        std::cout << " ";
        std::cout << std::setfill('0');

        for (int i = 1; i < b.size(); i++)
        {
            std::cout << " ";
            std::cout << std::setw(2) << std::hex << (static_cast<int>(b.get(i).asInt8()) & 0xFF);
        }
    }

    std::cout << std::endl;
}
