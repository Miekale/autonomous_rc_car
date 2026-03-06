#include "Serial.hpp"
#include <iostream>

// docs: https://en.wikibooks.org/wiki/Serial_Programming/termios 

// tut: https://chrisheydrick.com/2012/06/17/how-to-read-serial-data-from-an-arduino-in-linux-with-c-part-3/


Serial::Serial(const std::string& device, speed_t baudrate) 
    : device(device), baudrate(baudrate), fd(-1) {
    openPort();
}

bool Serial::openPort(){
    // These flags say that the line will be read/write and doensn't own the process
    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return false;
    }
    setupPortParams();
    return true;
}

Serial::~Serial() {
    closePort();
}

// Honestly just visible for testing purposes
bool Serial::isOpen() const {
    return !(fd == -1);
}

bool Serial::setupPortParams() {
    struct termios tty{};

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return false;
    }

    // I/O speeds, termios data structure called speed_t
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag |= (CLOCAL | CREAD);  
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8-bit chars
    tty.c_cflag &= ~PARENB;            // no parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;           // no flow control

    tty.c_lflag = 0;   
    tty.c_oflag = 0;
    // incase arduino sends random binary, disable flow so we don't interpret those as stop bits
    tty.c_iflag = 0;

    // block till at least one byte, and delay 0.1 second interbyte timeout
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1; // so if we don't get a new byte after 0.1s just return our last one

    tcflush(fd, TCIFLUSH);
    return tcsetattr(fd, TCSANOW, &tty) == 0;
}

void Serial::closePort() {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

int Serial::writeData(const std::string& data) {
    if (fd == -1) return -1;
    return write(fd, data.c_str(), data.size());
}

int Serial::writeData(const uint8_t command_id, const std::vector<float>& data) {
    /*
    Sets up a packet like
    0x0A 0x09 0x01 [8 bytes of data] checksum

    byte 1 is the header (always 0x0A)
    byte 2 is the length of the command id + number of data bytes
    byte 3 is the command id, see the Serial.hpp file for more info
    data bytes are optional
    checksum is an additional byte for validation
    */
    if (fd == -1) return -1;

    // 1 byte for CmdID + 4 bytes per float
    uint8_t len = 1 + (data.size() * sizeof(float)); 

    std::vector<uint8_t> packet;
    packet.reserve(2 + len + 1); // header, len byte, cmd + data, checksum

    packet.push_back(header); 
    packet.push_back(len);
    packet.push_back(command_id);

    // Copy each float into the packet
    for (float val : data) {
        uint8_t float_bytes[4];
        std::memcpy(float_bytes, &val, 4); 
        for (int i = 0; i < 4; i++) {
            packet.push_back(float_bytes[i]);
        }
    }

    // checksum of everything
    packet.push_back(getCheckSum(packet));
    
    return write(fd, packet.data(), packet.size());
}

uint8_t Serial::getCheckSum(const std::vector<uint8_t>& data) {
    uint8_t xor_ = 0;
    for (uint8_t val : data) {
        xor_ ^= val;
    }
    return xor_;
}

uint32_t Serial::getMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool Serial::waitForAck(uint32_t timeout_ms) {
    uint32_t start = getMillis();
    uint8_t resp = 0;

    while (getMillis() - start < timeout_ms) {

        if (read(fd, &resp, 1) > 0) { 
            if (resp == 0x06) return true; // ACK
            if (resp == 0x15) return false; // NAK
        }
        usleep(1000);
    }
    return false; 
}

bool Serial::sendWithRetry(uint8_t cmd, const std::vector<float>& data) {

    for (int attempt = 0; attempt < 3; attempt++) {
        std::cout << "b";
        if (writeData(cmd, data) < 0) {
            std::cout << "a";
            return false;
        }
        std::cout << "b";
        if (waitForAck(100))
            std::cout << "sink";
            return true;
    }
    return false;
}

int Serial::readData(char* buffer, size_t size) {
    if (fd == -1) return -1;

    /*
    Chat gave me this part because one of the test cases has that we simulate not
    sending any data from arduino to see what happens. Theres a case where reads should
    read the last byte if none were read recently, but it reads nothign if nothing has 
    ever been sent between the two. Poll just ensures we can account for this
    */
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    // Wait for 100ms
    int sel = poll(&pfd, 1, 100); 
    if (sel == 0) return 0;  // Timeout!
    if (sel < 0)  return -1; // Error

    return read(fd, buffer, size);
}


