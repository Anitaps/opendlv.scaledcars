#include <stdint.h>
#include <iostream>
#include <string>
#include <cstring>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "SerialReceiveBytes.hpp"

using namespace std;
string serial_received;
int ir1, ir2, ir3, uls, ometer;

void SerialReceiveBytes::nextString(const string &s) {
    char input[128];
    char IR_1[128];
    char IR_2[128];
    char IR_3[128];
    char US[128];
    char ODOM[128];
    


    serial_received += s;
    int head = serial_received.find('H');
    int tail = serial_received.find('T',head);

    if(head > -1 && tail > head){
    
       
        strcpy(input, serial_received.substr(head, tail - head + 1).c_str());
        cout << "Read input:" << input << endl;

       // H 12 a 32 b 20 c 12 d 20 T

        int IR1_H = serial_received.find('H');
        int IR1_T = serial_received.find('a');
//        int IR2_H = serial_received.find('a');
        int IR2_T = serial_received.find('b');
//        int IR3_H = serial_received.find('b');
        int IR3_T = serial_received.find('c');
//        int US_H = serial_received.find('c');
        int US_T = serial_received.find('d');
//        int Odom_H = serial_received.find('d');
        int Odom_T = serial_received.find('T');


        strcpy(IR_1, serial_received.substr(1+IR1_H, IR1_T - IR1_H -1).c_str());
        strcpy(IR_2, serial_received.substr(1+IR1_T, IR2_T - IR1_T-1).c_str());
        strcpy(IR_3, serial_received.substr(1+IR2_T, IR3_T - IR2_T-1).c_str());
        strcpy(US,   serial_received.substr(1+IR3_T, US_T - IR3_T-1).c_str());
        strcpy(ODOM, serial_received.substr(1+US_T, Odom_T - US_T-1).c_str());

        

        sscanf(IR_1, "%d", &ir1);
        sscanf(IR_2, "%d", &ir2);
        sscanf(IR_3, "%d", &ir3);
        sscanf(US, "%d", &uls);
        sscanf(ODOM, "%d", &ometer);
        


      //  cout << "IR_1:" << IR_1 << endl;
      //  cout << "IR_2:" << IR_2 << endl;
      //  cout << "IR_3:" << IR_3 << endl;
      //  cout << "US:" << US << endl;
      //  cout << "ODOM:" << ODOM << endl;

        serial_received = "";
    } else{

        cout << "Head and Tail not found in:" << serial_received << endl;
    }





/*

    for(int i=0; i<string.length(); i++){
        if(s.at(i) == ',', string ){

        }
        else{
            abc+=string(s.at(i));
        } 
    }
    */
    //cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
    
}

int SerialReceiveBytes::getir1(){
    
    return ir1;
}
int SerialReceiveBytes::getir2(){
    return ir2;
}
int SerialReceiveBytes::getir3(){
    return ir3;
}
int SerialReceiveBytes::getuls(){
    return uls;
}
int SerialReceiveBytes::getometer(){
    return ometer;
}
// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore;
using namespace odcore::wrapper;

/*int32_t main() {
    const string SERIAL_PORT = "/dev/pts/21";
    const uint32_t BAUD_RATE = 19200;

    // We are using OpenDaVINCI's std::shared_ptr to automatically
    // release any acquired resources.
    try {
        std::shared_ptr<SerialPort>
            serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

        // This instance will handle any bytes that are received
        // from our serial port.
        SerialReceiveBytes handler;
        serial->setStringListener(&handler);

        // Start receiving bytes.
        serial->start();

        const uint32_t ONE_SECOND = 1000 * 1000;
        odcore::base::Thread::usleepFor(120 * ONE_SECOND);

        // Stop receiving bytes and unregister our handler.
        serial->stop();
        serial->setStringListener(NULL);
    }
    catch(string &exception) {
        cerr << "Error while creating serial port: " << exception << endl;
    }
}*/