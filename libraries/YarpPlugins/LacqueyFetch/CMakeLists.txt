# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Author: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(LacqueyFetch
                    CATEGORY device
                    TYPE roboticslab::LacqueyFetch
                    INCLUDE LacqueyFetch.hpp
                    DEFAULT ON
                    DEPENDS ENABLE_CanBusSharerLib)

if(NOT SKIP_LacqueyFetch)

    if(NOT YARP_VERSION VERSION_GREATER_EQUAL 3.4)
        set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this
    endif()

    yarp_add_plugin(LacqueyFetch LacqueyFetch.hpp
                                 DeviceDriverImpl.cpp
                                 ICanBusSharerImpl.cpp
                                 IControlModeRawImpl.cpp
                                 IPWMControlRawImpl.cpp)

    target_link_libraries(LacqueyFetch YARP::YARP_os
                                       YARP::YARP_dev
                                       ROBOTICSLAB::CanBusSharerLib)

    target_compile_features(LacqueyFetch PUBLIC cxx_std_11)

    yarp_install(TARGETS LacqueyFetch
                 LIBRARY DESTINATION ${ROBOTICSLAB-YARP-DEVICES_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-YARP-DEVICES_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-YARP-DEVICES_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
