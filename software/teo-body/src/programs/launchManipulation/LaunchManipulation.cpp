// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchManipulation.hpp"

/************************************************************************/
LaunchManipulation::LaunchManipulation() { }

/************************************************************************/
bool LaunchManipulation::configure(ResourceFinder &rf) {

    //-- left arm --
    std::string leftArmIni = rf.findFileByName("leftArm.ini");

    Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"leftArm.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured left arm from  %s.\n",leftArmIni.c_str());
    leftArmOptions.put("name","/teo/leftArm");
    leftArmOptions.put("device","controlboard");
    leftArmOptions.put("subdevice","bodybot");

    leftArmDevice.open(leftArmOptions);
    
    if (!leftArmDevice.isValid()) {
        CD_ERROR("leftArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- right arm --
    std::string rightArmIni = rf.findFileByName("rightArm.ini");

    Property rightArmOptions;
    if (! rightArmOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightArm.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured right arm from %s.\n",rightArmIni.c_str());
    rightArmOptions.put("name","/teo/rightArm");
    rightArmOptions.put("device","controlboard");
    rightArmOptions.put("subdevice","bodybot");

    rightArmDevice.open(rightArmOptions);

    if (!rightArmDevice.isValid()) {
        CD_ERROR("rightArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    return true;
}

/************************************************************************/

bool LaunchManipulation::updateModule() {
    //printf("LaunchManipulation alive...\n");
    return true;
}

/************************************************************************/

bool LaunchManipulation::close() {
    leftArmDevice.close();
    rightArmDevice.close();
    return true;
}

/************************************************************************/
