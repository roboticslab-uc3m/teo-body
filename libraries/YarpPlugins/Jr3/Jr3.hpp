// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3__
#define __JR3__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <sstream>

#include <fcntl.h>  // ::open
#include <unistd.h>  // ::close

#include "jr3pci-ioctl.h"

#define DEFAULT_NUM_CHANNELS 24
#define DEFAULT_FILTER_ID 0

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * \defgroup Jr3
 * @brief Contains roboticslab::Jr3.
 */

 /**
 * @ingroup Jr3
 * @brief Implementation for the JR3 sensor. Launch as in: yarpdev --device Jr3 --period 20 --name /jr3:o
 *
 */
class Jr3 : public yarp::dev::DeviceDriver, public yarp::dev::IAnalogSensor
{

    public:

        Jr3()
            : fd(0)
        { }

        ~Jr3()
        { close(); }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IAnalogSensor Declarations. Implementation in IAnalogSensorImpl.cpp ---------
        /**
         * Read a vector from the sensor.
         * @param out a vector containing the sensor's last readings.
         * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
         */
        virtual int read(yarp::sig::Vector &out);

        /**
         * Check the state value of a given channel.
         * @param ch channel number.
         * @return status.
         */
        virtual int getState(int ch);

        /**
         * Get the number of channels of the sensor.
         * @return number of channels (0 in case of errors).
         */
        virtual int getChannels();

        /**
         * Calibrates the whole sensor.
         * @return status.
         */
        virtual int calibrateSensor();

        /**
         * Calibrates the whole sensor, using an vector of calibration values.
         * @param value a vector of calibration values.
         * @return status.
         */
        virtual int calibrateSensor(const yarp::sig::Vector& value);

        /**
         * Calibrates one single channel.
         * @param ch channel number.
         * @return status.
         */
        virtual int calibrateChannel(int ch);

        /**
         * Calibrates one single channel, using a calibration value.
         * @param ch channel number.
         * @param value calibration value.
         * @return status.
         */
        virtual int calibrateChannel(int ch, double value);

    private:
        void loadFilters(int id);

        six_axis_array fm0, fm1, fm2, fm3;
        force_array fs0, fs1, fs2, fs3;
        int fd;
        unsigned long int filters[4];

};

}  // namespace roboticslab

#endif  // __JR3__

