#include "Serial.hpp"


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


