option(ENABLE_DextraRawControlboardLib "Enable/disable DextraRawControlboardLib library" ON)

if(ENABLE_DextraRawControlboardLib)

    add_library(DextraRawControlboardLib SHARED DextraRawControlboard.hpp
                                                DextraRawControlboard.cpp
                                                IAxisInfoRawImpl.cpp
                                                IControlLimitsRawImpl.cpp
                                                IControlModeRawImpl.cpp
                                                IEncodersRawImpl.cpp
                                                IPositionControlRawImpl.cpp
                                                IPositionDirectRawImpl.cpp
                                                LogComponent.hpp
                                                LogComponent.cpp)

    target_link_libraries(DextraRawControlboardLib PUBLIC YARP::YARP_dev
                                                   PRIVATE YARP::YARP_os)

    target_include_directories(DextraRawControlboardLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    target_compile_features(DextraRawControlboardLib PUBLIC cxx_std_14)

    install(TARGETS DextraRawControlboardLib
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    add_library(ROBOTICSLAB::DextraRawControlboardLib ALIAS DextraRawControlboardLib)

else()

    set(ENABLE_DextraRawControlboardLib OFF CACHE BOOL "Enable/disable DextraRawControlboardLib library" FORCE)

endif()
