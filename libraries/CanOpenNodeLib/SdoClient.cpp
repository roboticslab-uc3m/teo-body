// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoClient.hpp"

#include <cstring>

#include <bitset>
#include <string>

#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>

#include "CanUtils.hpp"

YARP_LOG_COMPONENT(SDO, "rl.SdoClient")

using namespace roboticslab;

namespace
{
    std::string parseAbortCode(std::uint32_t code)
    {
        // CiA 301 v4.2.0
        switch (code)
        {
        case 0x05030000:
            return "Toggle bit not alternated";
        case 0x05040000:
            return "SDO protocol timed out";
        case 0x05040001:
            return "Client/server command specifier not valid or unknown";
        case 0x05040002:
            return "Invalid block size (block mode only)";
        case 0x05040003:
            return "Invalid sequence number (block mode only)";
        case 0x05040004:
            return "CRC error (block mode only)";
        case 0x05040005:
            return "Out of memory";
        case 0x06010000:
            return "Unsupported access to an object";
        case 0x06010001:
            return "Attempt to read a write only object";
        case 0x06010002:
            return "Attempt to write a read only object";
        case 0x06020000:
            return "Object does not exist in the object dictionary";
        case 0x06040041:
            return "Object cannot be mapped to the PDO";
        case 0x06040042:
            return "The number and length of the objects to be mapped would exceed PDO length";
        case 0x06040043:
            return "General parameter incompatibility reason";
        case 0x06040047:
            return "General internal incompatibility in the device";
        case 0x06060000:
            return "Access failed due to an hardware error";
        case 0x06070010:
            return "Data type does not match, length of service parameter does not match";
        case 0x06070012:
            return "Data type does not match, length of service parameter too high";
        case 0x06070013:
            return "Data type does not match, length of service parameter too low";
        case 0x06090011:
            return "Sub-index does not exist";
        case 0x06090030:
            return "Invalid value for parameter (download only)";
        case 0x06090031:
            return "Value of parameter written too high (download only)";
        case 0x06090032:
            return "Value of parameter written too low (download only)";
        case 0x06090036:
            return "Maximum value is less than minimum value";
        case 0x060A0023:
            return "Resource not available: SDO connection";
        case 0x08000000:
            return "General error";
        case 0x08000020:
            return "Data cannot be transferred or stored to the application";
        case 0x08000021:
            return "Data cannot be transferred or stored to the application because of local control";
        case 0x08000022:
            return "Data cannot be transferred or stored to the application because of the present device state";
        case 0x08000023:
            return "Object dictionary dynamic generation fails or no object dictionary is present (e.g. object "
                   "dictionary is generated from file and generation fails because of an file error)";
        case 0x08000024:
            return "No data available";
        default:
            return "unknown";
        }
    }
}

bool SdoClient::send(const std::uint8_t * msg)
{
    return sender && sender->prepareMessage({getCobIdRx(), 8, msg});
}

std::string SdoClient::msgToStr(std::uint16_t cob, const std::uint8_t * msgData)
{
    return CanUtils::msgToStr(id, cob, 8, msgData);
}

bool SdoClient::ping()
{
    std::uint8_t requestMsg[8] = {0x40}; // index: 0x0000, subindex: 0x00
    std::uint8_t responseMsg[8];
    return send(requestMsg) && stateObserver.await(responseMsg);
}

bool SdoClient::uploadInternal(const std::string & name, void * data, std::uint32_t size, std::uint16_t index, std::uint8_t subindex)
{
    std::uint8_t requestMsg[8] = {0};

    requestMsg[0] = 0x40; // client command specifier
    std::memcpy(requestMsg + 1, &index, 2);
    requestMsg[3] = subindex;

    std::uint8_t responseMsg[8];

    if (!performTransfer(name, requestMsg, responseMsg))
    {
        return false;
    }

    std::bitset<8> bitsReceived(responseMsg[0]);
    std::uint16_t expectedIndex;
    std::memcpy(&expectedIndex, responseMsg + 1, 2);

    if ((bitsReceived >> 5) != 2 || expectedIndex != index || responseMsg[3] != subindex)
    {
        yCError(SDO, "SDO client request (\"%s\") overrun (id %d)", name.c_str(), id);
        return false;
    }

    if (bitsReceived[1]) // expedited transfer
    {
        if (bitsReceived[0]) // data size is indicated in 'n'
        {
            const std::uint8_t n = ((bitsReceived << 4) >> 6).to_ulong();
            const std::uint8_t actualSize = 4 - n;

            if (size != actualSize)
            {
                yCError(SDO, "SDO client request (\"%s\") size mismatch: expected %u, got %u (id %d)", name.c_str(), size, actualSize, id);
                return false;
            }
        }

        std::memcpy(data, responseMsg + 4, size);
    }
    else
    {
        std::uint32_t len;
        std::memcpy(&len, responseMsg + 4, sizeof(len));

        if (size < len)
        {
            yCError(SDO, "SDO segmented upload (\"%s\") insufficient memory allocated: expected %u, got %u (id %d)", name.c_str(), len, size, id);
            return false;
        }

        yCInfo(SDO, "SDO segmented upload (\"%s\") begin (id %d)", name.c_str(), id);

        std::bitset<8> bitsSent(0x60);
        std::uint8_t segmentedMsg[8] = {0};
        std::uint32_t sent = 0;

        do
        {
            segmentedMsg[0] = bitsSent.to_ulong();

            if (!performTransfer(name, segmentedMsg, responseMsg))
            {
                return false;
            }

            bitsReceived = responseMsg[0];

            if ((bitsReceived >> 5) != 0)
            {
                yCError(SDO, "SDO segmented upload (\"%s\") overrun (id %d)", name.c_str(), id);
                return false;
            }

            if (bitsReceived[4] != bitsSent[4])
            {
                yCError(SDO, "SDO segmented upload (\"%s\") toggle bit mismatch (id %d)", name.c_str(), id);
                return false;
            }

            const std::uint8_t n = ((bitsReceived << 4) >> 5).to_ulong();
            const std::uint8_t actualSize = 7 - n;

            std::memcpy(static_cast<std::uint8_t *>(data) + sent, responseMsg + 1, actualSize);

            sent += actualSize;
            bitsSent.flip(4);
        }
        while (!bitsReceived[0]); // continuation bit

        yCInfo(SDO, "SDO segmented upload (\"%s\") end (id %d)", name.c_str(), id);
    }

    return true;
}

bool SdoClient::downloadInternal(const std::string & name, const void * data, std::uint32_t size, std::uint16_t index, std::uint8_t subindex)
{
    std::uint8_t indicationMsg[8] = {0};
    std::memcpy(indicationMsg + 1, &index, 2);
    indicationMsg[3] = subindex;

    std::bitset<8> indicationBits(0x21);
    std::bitset<8> bitsReceived;
    std::uint16_t expectedIndex;

    if (size <= 4) // expedited transfer
    {
        indicationBits.set(1); // e: transfer type
        const std::uint8_t n = 4 - size;
        indicationMsg[0] = indicationBits.to_ulong() + (n << 2);
        std::memcpy(indicationMsg + 4, data, size);

        std::uint8_t confirmMsg[8];

        if (!performTransfer(name, indicationMsg, confirmMsg))
        {
            return false;
        }

        bitsReceived = confirmMsg[0];
        std::memcpy(&expectedIndex, confirmMsg + 1, 2);

        if ((bitsReceived >> 5) != 3 || expectedIndex != index || confirmMsg[3] != subindex)
        {
            yCWarning(SDO, "SDO client indication (\"%s\") overrun (id %d)", name.c_str(), id);
            return false;
        }
    }
    else
    {
        indicationMsg[0] = indicationBits.to_ulong();
        std::memcpy(indicationMsg + 4, &size, sizeof(size));

        std::uint8_t confirmMsg[8];

        if (!performTransfer(name, indicationMsg, confirmMsg))
        {
            return false;
        }

        bitsReceived = confirmMsg[0];
        std::memcpy(&expectedIndex, confirmMsg + 1, 2);

        if ((bitsReceived >> 5) != 3 || expectedIndex != index || confirmMsg[3] != subindex)
        {
            yCError(SDO, "SDO client indication (\"%s\") overrun (id %d)", name.c_str(), id);
            return false;
        }

        std::bitset<8> bitsSent(0x00);
        std::uint32_t sent = 0;

        yCInfo(SDO, "SDO segmented download (\"%s\") begin (id %d)", name.c_str(), id);

        do
        {
            std::uint32_t actualSize;

            if (size - sent <= 7) // last message
            {
                actualSize = size - sent;
                bitsSent.set(0);
            }
            else
            {
                actualSize = 7;
            }

            const std::uint8_t n = 7 - actualSize;
            std::uint8_t segmentedMsg[8] = {0};
            segmentedMsg[0] = bitsSent.to_ulong() + (n << 1);

            std::memcpy(segmentedMsg + 1, static_cast<const std::uint8_t *>(data) + sent, actualSize);

            if (!performTransfer(name, segmentedMsg, confirmMsg))
            {
                return false;
            }

            bitsReceived = confirmMsg[0];

            if ((bitsReceived >> 5) != 1)
            {
                yCError(SDO, "SDO segmented download (\"%s\") overrun (id %d)", name.c_str(), id);
                return false;
            }

            if (bitsReceived[4] != bitsSent[4])
            {
                yCError(SDO, "SDO segmented download (\"%s\") toggle bit mismatch (id %d)", name.c_str(), id);
                return false;
            }

            sent += actualSize;
            bitsSent.flip(4);
        }
        while (!bitsSent[0]); // continuation bit

        yCInfo(SDO, "SDO segmented download (\"%s\") end (id %d)", name.c_str(), id);
    }

    return true;
}

bool SdoClient::upload(const std::string & name, std::string & s, std::uint16_t index, std::uint8_t subindex)
{
    const std::uint32_t maxLen = 100; // arbitrary high value
    char buf[maxLen] = {0};

    if (!uploadInternal(name, buf, maxLen, index, subindex))
    {
        return false;
    }

    s = buf;
    return true;
}

bool SdoClient::download(const std::string & name, const std::string & s, std::uint16_t index, std::uint8_t subindex)
{
    return downloadInternal(name, s.data(), s.size(), index, subindex);
}

bool SdoClient::performTransfer(const std::string & name, const std::uint8_t * req, std::uint8_t * resp)
{
    yCInfo(SDO, "SDO client transfer (\"%s\") %s", name.c_str(), msgToStr(cobRx, req).c_str());

    if (!send(req))
    {
        yCError(SDO, "SDO client request/indication (\"%s\") unable to send packet (id %d)", name.c_str(), id);
        return false;
    }

    if (!stateObserver.await(resp))
    {
        yCError(SDO, "SDO client request/indication (\"%s\") inactive/timeout (id %d)", name.c_str(), id);
        return false;
    }

    if (resp[0] == 0x80) // SDO abort transfer (ccs)
    {
        std::uint32_t code;
        std::memcpy(&code, resp + 4, sizeof(code));
        yCError(SDO, "SDO transfer abort (\"%s\"): %s (id %d)", name.c_str(), parseAbortCode(code).c_str(), id);
        return false;
    }

    yCInfo(SDO, "SDO server transfer (\"%s\") %s", name.c_str(), msgToStr(cobTx, resp).c_str());
    return true;
}
