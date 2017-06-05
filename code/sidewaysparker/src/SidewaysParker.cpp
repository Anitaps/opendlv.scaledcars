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
#include <cstdio>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "SidewaysParker.h"

#include "SerialReceiveBytes.hpp"
#include <math.h>
#include <stdint.h>                                                         //Serial
#include <iostream>                                                         //Serial
#include <string>                                                           //Serial
#include <memory>                                                           //Serial
#include <opendavinci/odcore/wrapper/SerialPort.h>                          //Serial
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>                   //Serial
#include <opendavinci/odcore/base/Thread.h>                                 //Serial

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUD_RATE 9600

namespace automotive {
     namespace miniature {
        //refering to inside libraries probably
        //all of these are from opendavinci and those others who are up are't from it
        using namespace std;      //standard function                        //Serial
        using namespace odcore;                                             //Serial
        using namespace odcore::wrapper;                                    //Serial
        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;
        double speed;
        double direction;
        std::shared_ptr<SerialPort> serial;
        //string test;

        SidewaysParker::SidewaysParker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SidewaysParker") {
        }

        SidewaysParker::~SidewaysParker() {}

        void SidewaysParker::setUp() {
            openSerialPort();
            // This method will be call automatically _before_ running body().

        }
       
//here we start the method for sending serial 
        void SidewaysParker::openSerialPort(){
            //const string SERIAL_PORT = "/dev/pts/19";        
            //const uint32_t BAUD_RATE = 9600;                    

        // We are using OpenDaVINCI's std::shared_ptr to automatically
        // release any acquired resources.
        try {   
                
                serial=std::shared_ptr<SerialPort>(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));  
                cout <<"Serial port opened " <<endl;
           
                
            }
        catch(string &exception) {
               cerr << "Serial port could not be created: " << exception << endl;
        }
        }


        void SidewaysParker::sendSerialData() {
            
                std::string steering = std::to_string(direction);
                std::string velocity = std::to_string(speed);
               
           
           
                serial->send("t" + velocity + "s" + steering + "\r\n");                       //("1550_"+serialSteering+"\r\n");
                
            }
       
        void SidewaysParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }
    
    

        // This method will do the main data processing job.
         odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {


            SerialReceiveBytes input;
            serial->setStringListener(&input);
            serial->start();

                const double INFRARED_FRONT_RIGHT = 0;
                const double INFRARED_REAR = 1;
                const double INFRARED_REAR_RIGHT = 2;
                const double ULTRASONIC_FRONT = 3;

            double size = 0;
            double irFRValue = 0;
            double start = 0;
            double end = 0;
            double a = 0;
            double b = 0; 
            double f = 0;
            double d = 0;
            double e = 0;
            double g = 0;
            double h = 0;
            double usF = 0;
            double irR = 0;
            double irFR = 0;
            double irRR = 0;

            int stageMeasuring = 0;
            int stageMoving = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
                   odcore::data::dmcp::ModuleStateMessage::RUNNING) {
               
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData>();

  
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData>();

                cout << "Please" << endl;
                cout << "Infrared" << input.getir1() << endl;
                cout << "INFRARED_REAR_RIGHT" << input.getir2() << endl;
                cout << "INFRARED_REAR" << input.getir3() << endl;
                cout << "ULTRASONIC_FRONT" << input.getuls() << endl;
                cout << "Odometer" << input.getometer() << endl;

                cout << "Work" << endl;
                
                VehicleControl vc;
                         

                    irR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR);                         
                    irRR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                    usF = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT);
                    irFR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                  
                  if (stageMoving == 0) {
                   
                    speed=1605;
                    direction=90;
                    sendSerialData();
                        // Go forward.
                        vc.setSpeed(2);
                        vc.setSteeringWheelAngle(0);
                        
                    }
                   
                    if (stageMoving == 1) {

                        speed=1500;
                        direction=90;
                        sendSerialData();
                        vc.setSpeed(0);
                        vc.setSteeringWheelAngle(0);
                        stageMoving = 2;
                        
                    }
                    if (stageMoving == 2) {

                        speed=1605;
                        direction=90;
                        sendSerialData();
                        vc.setSpeed(1);
                        if (irR < 0 && irRR > 0) {
                            stageMoving = 3;
                            a = vd.getAbsTraveledPath();
                            cout << "Stage Moving = 3" << endl;
                        }
                    }
                    if (stageMoving == 3 && (irR < 0)) {

                        speed=1605;
                        direction=90;
                        sendSerialData();

                        cout <<vd.getAbsTraveledPath() <<"size##################################### " << size << endl;
                                b = vd.getAbsTraveledPath();
                                vc.setSpeed(1);
                                vc.setSteeringWheelAngle(0);
                                    
                        if (b - a > 1.1087) {

                            speed=1400;
                            direction=115;         
                            sendSerialData();
                                vc.setSpeed(-1);
                                vc.setSteeringWheelAngle(25);
                            }

                       cout <<vd.getAbsTraveledPath() <<"**************" << endl;
                
                    if (b - a > 7.4432) {
                        stageMoving = 4;
                    }   
                    }
                    if (stageMoving == 4) {

                        speed=1400;
                        direction=65;
                        sendSerialData();

                        vc.setSpeed(-1);
                        vc.setSteeringWheelAngle(-25);

                        if (irR > 0 && irR < 2.5 ) {
                         stageMoving = 5;
                    }
                   
                    }
                   
                    
                    if (stageMoving == 5) {

                        speed=1605;
                        direction=115;
                        sendSerialData();
                        vc.setSpeed(1);
                        vc.setSteeringWheelAngle(25);
                    
                        if(usF < 2 && usF > 0){
                         stageMoving = 6;
                    }
}                       
                        if (stageMoving == 6) {

                            speed=1400;
                            direction=65;
                            sendSerialData();

                          vc.setSpeed(-1);
                          vc.setSteeringWheelAngle(-25);

                        if (irR > 0 && irR < 2.5) {
                         stageMoving = 7;
                        cout << "lasttttttttttttttttttt!" << endl;
                    }
                    }
                    if (stageMoving == 7) {

                        speed=1500;
                        direction=90;
                        sendSerialData();
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                }
                    
                    if (stageMoving == 8){

                        speed=1500;
                        direction=90;
                        sendSerialData();
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    f = vd.getAbsTraveledPath();
                     stageMoving = 9;
                }
                
                    if (stageMoving == 9) {

                        speed=1605;
                        direction=90;
                        sendSerialData();

                    d = vd.getAbsTraveledPath();
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(0);
                    if (d - f > 3.1087 && d - f < 9.4432 ) {
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(25);
                }
                    if (d - f > 9.4432){
                     stageMoving = 10;
                    }
                }
                if (stageMoving == 10) {

                    speed=1400;
                    direction=65;
                    sendSerialData();

                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-25);
                    e = vd.getAbsTraveledPath();
                    if (e - d > 3){
                     stageMoving = 11;
                    }
                }
                if (stageMoving == 11) {

                    speed=1605;
                    direction=115;
                    sendSerialData();

                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(25);
                    g = vd.getAbsTraveledPath();
                    if (g - e > 3){
                     stageMoving = 12;
                    }
                }
                if (stageMoving == 12) {

                    speed=1400;
                    direction=65;
                    sendSerialData();

                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-25);
                    h = vd.getAbsTraveledPath();
                    if (h - g > 1){
                     stageMoving = 7;
                    }
                }
                if (stageMoving == 0 ) {
                    switch (stageMeasuring) {
                        
                        case 0: {
                            irFRValue = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            double parking = vd.getAbsTraveledPath();
                            start = vd.getAbsTraveledPath();
                            if (irFR < 0  && parking >= 7.5) {
                                size = parking;
                                stageMoving = 8;
                                stageMeasuring++;
                            }
                            if (irFR > 0 && parking < 7.5) {
                                stageMeasuring++;
                            }
                        }
                            break;
                        
                        case 1: {
                                if ((irFRValue > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0)) {
                                    cout << "Setting stageMeasuring to 2 " << endl;
                                    stageMeasuring = 2;
                                    start = vd.getAbsTraveledPath();
                                }
                                irFRValue = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                            
                        }
                            break;
                        
                        case 2: {
                                end = vd.getAbsTraveledPath();
                            if (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 0 && end - start >= 7.5){
                                size = end - start;
                                stageMoving = 8;
                                stageMeasuring++;                       
                                }
                            if (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0 && end - start < 7.5) {
                                size = end - start;
                                cout << "Sizeeee = " << size << endl;
                               
                                stageMeasuring = 1;
                                if ((stageMoving < 1) &&
                                    (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (size > 7)) {
                                    stageMeasuring = 1;
                                    stageMoving = 1;
                                    cout << "Sizeeee = " << size << endl;
                                }


                            }
                            irFRValue = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        }
                            break;
                    }
                }  
                //this iswhere container is created with vehicle data,speed direction etc
                Container c(vc);  //vc is vehicle controller,, we create a container which its name is c and c contains vc 
                // Send container.
                getConference().send(c);
            }
            serial ->stop();
            serial ->setStringListener(NULL);

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature
