// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "MotorIpos.hpp"

// -----------------------------------------------------------------------------

MotorIpos::MotorIpos(CanBusHico *canDevicePtr, const int& canId, const double &tr) : max(0), min(0), refAcceleration(0), refSpeed(0), encoder(0) {
    this->canDevicePtr = canDevicePtr;
    this->canId = canId;
    this->tr = tr;
    this->ptPointCounter = 0;
    this->ptMovementDone = false;
    this->targetReached = false;
    CD_SUCCESS("Created driver with canId %d and tr %f, pointer to CAN bus device, and all local parameters set to 0.\n",canId,tr);
}

// -----------------------------------------------------------------------------

int MotorIpos::getCanId() {
    return canId;
}

// -----------------------------------------------------------------------------

double MotorIpos::getEncoder() {
    double value;
    encoderReady.wait();
    value = encoder;
    encoderReady.post();
    return value;
}

// -----------------------------------------------------------------------------

double MotorIpos::getMax() {
    return max;
}

// -----------------------------------------------------------------------------

double MotorIpos::getMin() {
    return min;
}

// -----------------------------------------------------------------------------

double MotorIpos::getRefAcceleration() {
    return refAcceleration;
}

// -----------------------------------------------------------------------------

double MotorIpos::getRefSpeed() {
    return refSpeed;
}

// -----------------------------------------------------------------------------

double MotorIpos::getTr() {
    return tr;
}

// -----------------------------------------------------------------------------

void MotorIpos::setEncoder(const double& value) {
    encoderReady.wait();
    encoder = value;
    encoderReady.post();
}

// -----------------------------------------------------------------------------

void MotorIpos::setMax(const double& value) {
    max = value;
}

// ----------------------------.-------------------------------------------------

void MotorIpos::setMin(const double& value) {
    min = value;
}

// -----------------------------------------------------------------------------

void MotorIpos::setRefAcceleration(const double& value) {
    refAcceleration = value;
}

// -----------------------------------------------------------------------------

void MotorIpos::setRefSpeed(const double& value) {
    refSpeed = value;
}

// -----------------------------------------------------------------------------

void MotorIpos::setTr(const double& value) {
    tr = value;
}

// -----------------------------------------------------------------------------

bool MotorIpos::send(uint32_t cob, uint16_t len, uint8_t * msgData) {

    if ( (lastUsage - yarp::os::Time::now()) < DELAY )
        yarp::os::Time::delay( lastUsage + DELAY - yarp::os::Time::now() );

    if( ! canDevicePtr->sendRaw(cob + this->canId, len, msgData) )
        return false;

    lastUsage = yarp::os::Time::now();
    return true;
}

// -----------------------------------------------------------------------------

bool MotorIpos::start() {

    //*************************************************************
    uint8_t msg_start[] = {0x01,0x01};

    msg_start[1]=this->getCanId();
    if( ! canDevicePtr->sendRaw(0, 2, msg_start) )
    {
        CD_ERROR("Could not send \"start\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"start\" to canId: %d.\n",this->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorIpos::readyToSwitchOn() {

    //*************************************************************
    uint8_t msg_readyToSwitchOn[] = {0x06,0x00}; //-- readyToSwitchOn, also acts as shutdown.

    if( ! this->send( 0x200, 2, msg_readyToSwitchOn) )
    {
        CD_ERROR("Could not send \"readyToSwitchOn/shutdown\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"readyToSwitchOn/shutdown\" to canId: %d.\n",this->getCanId());
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorIpos::switchOn() {

    //*************************************************************
    uint8_t msg_switchOn[] = {0x07,0x00};  //-- switchOn, also acts as disableOperation
    if( ! this->send( 0x200, 2, msg_switchOn) )
    {
        CD_ERROR("Could not send \"switchOn/disableOperation\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"switchOn/disableOperation\"  to canId: %d. Should respond...\n",this->getCanId());
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    yarp::os::Time::delay(0.1);
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    struct can_msg replyOn;
    while(canDevicePtr->read_timeout(&replyOn,20))
    {
        CD_SUCCESS("Got response to \"switchOn/disableOperation\" from canId: %d.\n",replyOn.id & 0x7F);
    }
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorIpos::enable() {

    //*************************************************************
    uint8_t msg_enable[] = {0x0F,0x00}; // enable

    if( ! this->send( 0x200, 2, msg_enable) )
    {
        CD_ERROR("Could not send \"enable\" to canId: %d.\n",this->getCanId());
        return false;
    }
    CD_SUCCESS("Sent \"enable\" to canId: %d. Should respond...\n",this->getCanId());
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    yarp::os::Time::delay(0.1);
    //* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    struct can_msg reply;
    while(canDevicePtr->read_timeout(&reply,20))
    {
        CD_SUCCESS("Got response to \"enable\" from canId: %d.\n",reply.id & 0x7F);
    }
    //*************************************************************

    return true;
}

// -----------------------------------------------------------------------------

bool MotorIpos::interpretMessage( can_msg * message) {

    if( (message->data[1]==0x64) && (message->data[2]==0x60) )
    {
        //-- Commenting encoder value (response to petition) as way too verbose, happens all the time.
        //CD_DEBUG("Is encoder value (response to petition).\n");
        int got;
        memcpy(&got, message->data+4,4);
        setEncoder( got / ( 11.11112 * getTr() ) );
        return true;
    }

    if( (message->data[0]==0x01)&&(message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[4]==0x20) )
    {
        ptBuffer.wait();
        CD_WARNING("pt buffer full (%d)!\n",canId);
        return true;
    }

    if( (message->data[0]==0x01)&&(message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[4]==0x00) )
    {
        ptBuffer.post();
        CD_WARNING("pt buffer empty (%d).\n",canId);
        return true;
    }

    if( (message->data[0]==0x01)&&(message->data[1]==0xFF)&&(message->data[2]==0x01)&&(message->data[3]==0x08) )
    {
        CD_WARNING("pt buffer message (don't know what it means) at canId %d.\n",canId);
        return true;
    }

    if( (message->data[0]==0x37)&&(message->data[1]==0x96) )
    {
        CD_WARNING("ended movement (canId %d).\n",canId);
        ptMovementDone = true;
        return true;
    }

    if( (message->data[1]==0x41)&&(message->data[2]=0x60) )
    {
        if(message->data[5] & 4) {
            targetReached = true;
            CD_DEBUG("\t-Target reached (%d).\n",canId);
        }
        else
        {
            targetReached = false;
            CD_DEBUG("\t-Target NOT reached (%d).\n",canId);
        }
        return true;
    }

    //-- error stuff

    //-- no meaning in next line
    //if (message->id>=0x80 && message->id<0x100) return true;

    if (message->data[0] == 0x01 && message->data[1] == 0xFF)
    {
        CD_DEBUG("PVT control message.\n");
        return true; // PVT control message
    }

    CD_ERROR("Emergency message in id: %d. ", message->id & 0x7F );

    printf("%X %X : ",message->data[1],message->data[0]);

    switch (message->data[1]){
        case 0:
            switch(message->data[0]){
                case 0: CD_ERROR("Error Reset\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x10:
            switch(message->data[0]){
                case 0: CD_ERROR("Generic error\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x23:
            switch(message->data[0]){
                case 0x10: CD_ERROR("Continuous over-current\n");
                break;
                case 0x40: CD_ERROR("Short-circuit\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x32:
            switch(message->data[0]){
                case 0x10: CD_ERROR("DC-link over-voltage\n");
                break;
                case 0x20: CD_ERROR("DC-link under-voltage\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x42:
            switch(message->data[0]){
                case 0x80: CD_ERROR("Over temperature motor\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x43:
            switch(message->data[0]){
                case 0x10: CD_ERROR("Over temperature drive.\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x54:
            switch(message->data[0]){
                case 0x41: CD_ERROR("Driver disabled due to enable input.\n");
                break;
                case 0x42: CD_ERROR("Negative limit switch active.\n");
                break;
                case 0x43: CD_ERROR("Positive limit switch active.\n");
                break;
                default: CD_ERROR("Unknown error.\n");
            };
        break;
        case 0x61:
            switch(message->data[0]){
                case 0x00: CD_ERROR("Invalid stup data.\n");
                break;
                default: CD_ERROR("Unknown error.\n");
            };
        break;
        case 0x75:
            switch(message->data[0]){
                case 0x00: CD_ERROR("Communication error.\n");
                break;
                default: CD_ERROR("Unknown error.\n");
            };
        break;
        case 0x81:
            switch(message->data[0]){
                case 0x10: CD_ERROR("CAN overrun (message lost).\n");
                break;
                case 0x30: CD_ERROR("Life guard error or heartbeat error.\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x83:
            switch(message->data[0]){
                case 0x31: CD_ERROR("I2t protection triggered.\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x85:
            switch(message->data[0]){
                case 0x80: CD_ERROR("Position wraparound / Hal sensor missing.\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x86:
            switch(message->data[0]){
                case 0x11: CD_ERROR("Control error / Following error.\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0x90:
            switch(message->data[0]){
                case 0x00: CD_ERROR("Command error\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        case 0xFF:
            switch(message->data[0]){
                case 0x01: CD_ERROR("Generic interpolated position mode error ( PVT / PT error.\n");
                break;
                case 0x02: CD_ERROR("Change set acknowledge bit wrong value.\n");
                break;
                case 0x03: CD_ERROR("Specified homing method not available.\n");
                break;
                case 0x04: CD_ERROR("A wrong mode is set in object 6060h, modes_of_operation.\n");
                break;
                case 0x05: CD_ERROR("Specified digital I/O line not available.\n");
                break;
                default: CD_ERROR("Unknown error\n");
            };
        break;
        default: CD_ERROR("Unknown error\n");
    }

}  //-- ends interpretMessage

// -----------------------------------------------------------------------------