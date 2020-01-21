// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <cstring>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "YarpCanSenderDelegate.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void CanReaderWriterThread::beforeStart()
{
    CD_INFO("Initializing CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str());
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::afterStart(bool success)
{
    CD_INFO("Configuring CanBusControlboard %s thread %s... %s\n", type.c_str(), id.c_str(), success ? "success" : "failure");
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::onStop()
{
    CD_INFO("Stopping CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str());
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::setDelay(double delay)
{
    this->delay = delay <= 0.0 ? std::numeric_limits<double>::min() : delay;
}

// -----------------------------------------------------------------------------

CanReaderThread::CanReaderThread(const std::string & id)
    : CanReaderWriterThread("read", id)
{}

// -----------------------------------------------------------------------------

void CanReaderThread::registerHandle(ICanBusSharer * p)
{
    canIdToHandle[p->getId()] = p;

    for (auto id : p->getAdditionalIds())
    {
        canIdToHandle[id] = p;
    }
}

// -----------------------------------------------------------------------------

void CanReaderThread::run()
{
    unsigned int read;
    bool ok;

    while (!isStopping())
    {
        //-- Lend CPU time to write threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(delay);

        //-- Return immediately if there is nothing to be read (non-blocking call), return false on errors.
        ok = iCanBus->canRead(canBuffer, bufferSize, &read);

        //-- All debugging messages should be contained in canRead, so just loop again.
        if (!ok || read == 0) continue;

        for (int i = 0; i < read; i++)
        {
            const yarp::dev::CanMessage & msg = canBuffer[i];
            auto it = canIdToHandle.find(msg.getId() & 0x7F);

            if (it != canIdToHandle.end())
            {
                it->second->notifyMessage({msg.getId(), msg.getLen(), msg.getData()});
            }
        }
    }
}

// -----------------------------------------------------------------------------

CanWriterThread::CanWriterThread(const std::string & id)
    : CanReaderWriterThread("write", id),
      sender(nullptr),
      preparedMessages(0)
{}

// -----------------------------------------------------------------------------

CanWriterThread::~CanWriterThread()
{
    delete sender;
}

// -----------------------------------------------------------------------------

void CanWriterThread::flush()
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    //-- Nothing to write, exit.
    if (preparedMessages == 0) return;

    yarp::dev::CanErrors errors;

    //-- Query bus state.
    if (!iCanBusErrors->canGetErrors(errors) || errors.busoff)
    {
        //-- Bus off, reset TX queue.
        preparedMessages = 0;
        return;
    }

    unsigned int sent;

    //-- Write as many bytes as possible, return false on errors.
    if (!iCanBus->canWrite(canBuffer, preparedMessages, &sent))
    {
        //-- Something bad happened, try again on the next call.
        return;
    }

    //-- Some messages could not be sent, preserve them for later.
    if (sent != preparedMessages)
    {
        handlePartialWrite(sent);
    }

    preparedMessages -= sent;
}

// -----------------------------------------------------------------------------

void CanWriterThread::run()
{
    while (!isStopping())
    {
        //-- Lend CPU time to read threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(delay);

        //-- Send everything and reset the queue.
        flush();
    }
}

// -----------------------------------------------------------------------------

void CanWriterThread::handlePartialWrite(unsigned int sent)
{
    for (int i = sent, j = 0; i < preparedMessages; i++, j++)
    {
        yarp::dev::CanMessage & msg = canBuffer[j];
        const yarp::dev::CanMessage & pendingMsg = canBuffer[i];

        msg.setId(pendingMsg.getId());
        msg.setLen(pendingMsg.getLen());
        std::memcpy(msg.getData(), pendingMsg.getData(), pendingMsg.getLen());
    }
}

// -----------------------------------------------------------------------------

void CanWriterThread::setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBusErrors * iCanBusErrors,
        yarp::dev::ICanBufferFactory * iCanBufferFactory, unsigned int bufferSize)
{
    CanReaderWriterThread::setCanHandles(iCanBus, iCanBusErrors, iCanBufferFactory, bufferSize);
    sender = new YarpCanSenderDelegate(canBuffer, bufferMutex, preparedMessages, bufferSize);
}

// -----------------------------------------------------------------------------

CanSenderDelegate * CanWriterThread::getDelegate()
{
    return sender;
}

// -----------------------------------------------------------------------------
