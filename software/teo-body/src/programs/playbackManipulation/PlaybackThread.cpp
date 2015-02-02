// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

/************************************************************************/

bool PlaybackThread::threadInit() {

    leftArmDone = false;
    rightArmDone = false;

    return true;

}

/************************************************************************/
void PlaybackThread::run() {

    CD_DEBUG("Begin parsing file.\n" );
    std::string line;
    while( getline( ifs, line) && ( ! this->isStopping() ) ) {

        yarp::os::Bottle lineBottle(line);  //-- yes, using a bottle to parse a string
        CD_DEBUG("string from bottle from string: %s\n", lineBottle.toString().c_str() );
        if( leftArmNumMotors+rightArmNumMotors != lineBottle.size() )
            CD_ERROR("-------------SIZE!!!!!!!!!!!!!\n");

        std::vector< double > leftArmOutDoubles( leftArmNumMotors );
        for(int i=0;i<leftArmNumMotors;i++)
            leftArmOutDoubles[i] = lineBottle.get(i).asDouble();

        std::vector< double > rightArmOutDoubles( rightArmNumMotors );
        for(int i=0;i<rightArmNumMotors;i++)
            rightArmOutDoubles[i] = lineBottle.get(leftArmNumMotors+i).asDouble();

        leftArmPosDirect->setPositions( leftArmOutDoubles.size(), NULL, leftArmOutDoubles.data() );
        rightArmPosDirect->setPositions( rightArmOutDoubles.size(), NULL, rightArmOutDoubles.data() );

    }
    if( this->isStopping() ) return;
    CD_DEBUG("End parsing file.\n" );


    while( (! leftArmDone) &&  ( ! this->isStopping() )) {
        CD_DEBUG("Left arm not done!\n");
        leftArmPos->checkMotionDone(&leftArmDone);
        yarp::os::Time::delay(0.1);  //-- [s]
    }
    if( this->isStopping() ) return;
    CD_DEBUG("Left arm done!\n");

    while( (! rightArmDone) &&  ( ! this->isStopping() )) {
        CD_DEBUG("Right arm not done!\n");
        rightArmPos->checkMotionDone(&rightArmDone);
        yarp::os::Time::delay(0.1);  //-- [s]
    }
    if( this->isStopping() ) return;
    CD_DEBUG("Right arm done!\n");

}


/************************************************************************/