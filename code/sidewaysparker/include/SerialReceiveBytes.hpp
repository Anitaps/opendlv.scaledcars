#include <opendavinci/odcore/io/StringListener.h>

// This class will handle the bytes received via a serial link.
class SerialReceiveBytes : public odcore::io::StringListener {
public:
    // Your class needs to implement the method void nextString(const std::string &s).
    virtual void nextString(const std::string &s);
    int getir1();
    int getir2();
    int getir3();
    int getuls();
    int getometer();
//	string abc;
};
