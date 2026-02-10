#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

class Serial {
public:
    Serial(const std::string& device, speed_t baudrate);
    ~Serial();
    void closePort();
    int writeData(const std::string& data);
    int readData(char* buffer, size_t size);

    bool isOpen() const;

private:
    int fd;
    std::string device;
    speed_t baudrate;

    bool setupPortParams();
    bool openPort();
};