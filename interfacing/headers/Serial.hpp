#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <cstring> 

// Command IDs
enum command_id_type : uint8_t {
    CMD_PURE_PURSUIT  = 0x00,
    CMD_CLAW_OPEN     = 0x01,
    CMD_CLAW_CLOSE    = 0x02,
    CMD_STOP          = 0x03  
};

class Serial {
public:
    Serial(const std::string& device, speed_t baudrate);
    ~Serial();
    void closePort();
    int writeData(const std::string& data);
    int writeData(const uint8_t command_id, const std::vector<float>& data);
    int readData(char* buffer, size_t size);

    bool isOpen() const;

private:
    int fd;
    std::string device;
    speed_t baudrate;
    uint8_t header = 0xAA;

    bool setupPortParams();
    bool openPort();
    uint8_t getCheckSum(const std::vector<uint8_t>& data);
};

#endif