# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# YARP devices.
add_subdirectory(AmorControlboard)
add_subdirectory(AravisGigE)
add_subdirectory(CanBusControlboard)
add_subdirectory(CanBusFake)
add_subdirectory(CanBusHico)
add_subdirectory(CanBusPeak)
add_subdirectory(CanBusSocket)
add_subdirectory(CuiAbsolute)
add_subdirectory(DextraCanControlboard)
add_subdirectory(DextraSerialControlboard)
add_subdirectory(EmulatedControlboard)
add_subdirectory(FakeJoint)
add_subdirectory(JointCalibrator)
add_subdirectory(Jr3)
add_subdirectory(LacqueyFetch)
add_subdirectory(LeapMotionSensor)
add_subdirectory(PhidgetSpatial)
add_subdirectory(SpaceNavigator)
add_subdirectory(TechnosoftIpos)
add_subdirectory(TextilesHand)
add_subdirectory(WiimoteSensor)

# Port monitor plugins.
add_subdirectory(PortMonitorPlugins)
