#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "SerialReceiveBytes.hpp"

using namespace std;

void SerialReceiveBytes::nextString(const string &s) {
    cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
}

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace odcore;
using namespace odcore::wrapper;

void SerialReceiveBytes::receiveSerial() {
    const string SERIAL_PORT = "/dev/pts/19";
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
        odcore::base::Thread::usleepFor(10 * ONE_SECOND);

        // Stop receiving bytes and unregister our handler.
        serial->stop();
        serial->setStringListener(NULL);
    }
    catch(string &exception) {
        cerr << "Error while creating serial port: " << exception << endl;
    }
}
