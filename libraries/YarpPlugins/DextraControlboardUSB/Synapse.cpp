// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Synapse.hpp"

#include <cstring>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    const char HEADER = 0x7E;
    const char FOOTER = 0x7E;
    const int FLOAT_SIZE = 4;
    const int MESSAGE_SIZE = (FLOAT_SIZE + 1) * Synapse::DATA_POINTS;
    const char FINGER_ADDRESS[Synapse::DATA_POINTS] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
}

Synapse::Synapse()
    : iSerialDevice(0)
{}

bool Synapse::getMessage(unsigned char * msg)
{
    char chr;
    while (!iSerialDevice->receiveChar(chr) || chr != HEADER) {}
    return iSerialDevice->receiveBytes(msg, MESSAGE_SIZE + 2) != 0;
}

bool Synapse::readDataList(std::vector<double> & setpoints)
{
    unsigned char msg[MESSAGE_SIZE + 2]; // data (MESSAGE_SIZE) + check (1) + footer (1)

    if (!getMessage(msg))
    {
        return false;
    }

    int i = 0;

    for (int j = 0; j < DATA_POINTS; j++)
    {
        if (msg[i] = FINGER_ADDRESS[j])
        {
            i++;
            double setpoint;
            std::memcpy(&setpoint, msg + i, FLOAT_SIZE);
            i += FLOAT_SIZE;
			setpoints[j] = setpoint;
        }
    }

    return true;
}

bool Synapse::writeSetpointList(const std::vector<double> & setpoints)
{
    char check = 0x00;
    char msg[MESSAGE_SIZE + 3]; // header (1) + data (MESSAGE_SIZE) + check (1) + footer (1)

    msg[0] = HEADER;

    for (int i = 0; i < DATA_POINTS; i++)
    {
        int j = 1 + (FLOAT_SIZE + 1) * i;
        msg[j] = FINGER_ADDRESS[i];
        std::memcpy(msg + j + 1, &setpoints[i], FLOAT_SIZE);
        check ^= FINGER_ADDRESS[i];

        for (int k = 0; k < FLOAT_SIZE; k++)
        {
            check ^= msg[j + 1 + k];
        }
    }

    msg[MESSAGE_SIZE - 2] = check;
    msg[MESSAGE_SIZE - 1] = FOOTER;

    return iSerialDevice->send(msg, MESSAGE_SIZE);
}