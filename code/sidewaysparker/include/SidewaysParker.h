/**
 * sidewaysparker - Sample application for realizing a sideways parking car.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef SIDEWAYSPARKER_H_
#define SIDEWAYSPARKER_H_

#include <stdint.h>                                                         //Serial
#include <iostream>                                                         //Serial
#include <string>                                                           //Serial
#include <memory>                                                           //Serial
#include <opendavinci/odcore/wrapper/SerialPort.h>                          //Serial
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>                   //Serial


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opendavinci/odcore/base/Thread.h>                                 //Serial

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"


#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
//#include "SerialReceiveBytes.cpp"

namespace automotive {
    namespace miniature {

        using namespace std;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class SidewaysParker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                SidewaysParker(const SidewaysParker &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                SidewaysParker& operator=(const SidewaysParker &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                SidewaysParker(const int32_t &argc, char **argv);

                virtual ~SidewaysParker();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            private:
                virtual void setUp();
		void openSerialPort();
               	void sendSerialData();
                virtual void tearDown();

        };

    }
} // automotive::miniature

#endif /*SIDEWAYSPARKER_H_*/
