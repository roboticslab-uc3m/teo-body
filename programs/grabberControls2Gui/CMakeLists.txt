if(Python_VERSION VERSION_LESS 3.5 AND (NOT DEFINED ENABLE_grabbercontrols2gui OR ENABLE_grabbercontrols2gui))
    message(WARNING "Python interpreter not found or unsupported version (only 3.5+), disabling grabbercontrols2gui program")
endif()

cmake_dependent_option(ENABLE_grabbercontrols2gui "Enable/disable grabbercontrols2gui program" ON
                       "Python_FOUND;NOT Python_VERSION VERSION_LESS 3.5" OFF)

if(ENABLE_grabbercontrols2gui)

    # configure grabbercontrols2gui setup.py
    configure_file("${CMAKE_SOURCE_DIR}/cmake/templates/setup.py.in"
                   "${CMAKE_BINARY_DIR}/setup.py")

    # run setup.py (install rule)
    install(CODE "EXECUTE_PROCESS(COMMAND \"${Python_EXECUTABLE}\" \"${CMAKE_BINARY_DIR}/setup.py\" install)")

else()

    set(ENABLE_grabbercontrols2gui OFF CACHE BOOL "Enable/disable grabbercontrols2gui program" FORCE)

endif()
