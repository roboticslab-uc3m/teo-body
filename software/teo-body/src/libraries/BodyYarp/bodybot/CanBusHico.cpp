// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusHico.hpp"

// -----------------------------------------------------------------------------

bool teo::CanBusHico::init(const std::string devicePath, const int bitrate) {

    //-- Open the CAN device for reading and writing.
    fileDescriptor = open(devicePath.c_str(), O_RDWR);
    if(fileDescriptor<0)
    {
        CD_ERROR("Could not open CAN device of path: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("Opened CAN device of path: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Set the CAN bitrate.
    if( ioctl(fileDescriptor,IOC_SET_BITRATE,&bitrate) != 0)
    {
        CD_ERROR("Could not set bitrate on CAN device: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("Bitrate set on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    //-- Start the CAN device.
    if( ioctl(fileDescriptor,IOC_START) != 0)
    {
        CD_ERROR("IOC_START failed on CAN device: %s\n", devicePath.c_str());
        return false;
    }
    CD_SUCCESS("IOC_START ok on CAN device: %s\n", devicePath.c_str());

    yarp::os::Time::delay(DELAY);

    return true;
}

// -----------------------------------------------------------------------------

bool teo::CanBusHico::close() {
    //release semaphore?
    ::close(fileDescriptor);
}

// -----------------------------------------------------------------------------

bool teo::CanBusHico::sendRaw(uint32_t id, uint16_t len, uint8_t * msgData) {

     struct can_msg msg;
     memset(&msg,0,sizeof(struct can_msg));
     msg.ff = FF_NORMAL;
     msg.id = id;
     msg.dlc = len;
     memcpy(msg.data, msgData, len*sizeof(uint8_t));

     canBusReady.wait();
     if ( write(fileDescriptor,&msg,sizeof(struct can_msg)) == -1)
     {
         canBusReady.post();
         CD_ERROR("%s.\n", strerror(errno))
         return false;
     }
     canBusReady.post();

     return true;
}

// -----------------------------------------------------------------------------

int teo::CanBusHico::read_timeout(struct can_msg *buf, unsigned int timeout) {

    fd_set fds;
    struct timeval tv;
    int sec,ret;
    FD_ZERO(&fds);

    sec=timeout/1000;
    tv.tv_sec=sec;
    tv.tv_usec=(timeout-(sec*1000))*1000;

    FD_SET(fileDescriptor,&fds);

    //-- select() returns the number of ready descriptors, or -1 for errors.
    ret=select(fileDescriptor+1,&fds,0,0,&tv);
    if(ret==0)
    {
        //-- Commenting select() timeout as way too verbose, happens all the time.
        //CD_DEBUG("select() timeout.\n");
        return 0;  // Return 0 on select timeout.
    }
    else if (ret<0)
    {
        CD_ERROR("select() error: %s.\n", strerror(errno));
        return ret;   // Return <0 on select error.
    }

    assert(FD_ISSET(fileDescriptor,&fds));

    canBusReady.wait();
    //-- read() returns the number read, -1 for errors or 0 for EOF.
    ret=read(fileDescriptor,buf,sizeof(struct can_msg));
    canBusReady.post();

    if (ret<0)
    {
        CD_ERROR("read() error: %s.\n", strerror(errno));
    }

    return ret;  //-- If gets to here, return whatever read() returned.
}

// -----------------------------------------------------------------------------

