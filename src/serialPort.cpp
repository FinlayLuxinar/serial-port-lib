//----------------------------------------------------------------------------------------------
// INCLUDE SELF
//----------------------------------------------------------------------------------------------
#include "serialPort.h"

//----------------------------------------------------------------------------------------------
// STANDARD INCLUDES
//----------------------------------------------------------------------------------------------
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
#include <termios.h>
#include <sys/select.h>

//----------------------------------------------------------------------------------------------
// PRIVATE DEFINES
//----------------------------------------------------------------------------------------------
#define SHOWEXTENDEDSTATUS true

//----------------------------------------------------------------------------------------------
// IMPLEMENTATION
//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------------------------------
/**
 * @class SerialPort
 * @brief Represents a serial port interface for communication with external devices.
 *
 * This class encapsulates the functionality to initialize, configure, read from, 
 * and write to a serial port. It uses termios for configuring port parameters
 * and handles common serial port operations like sending and receiving data.
 */

//----------------------------------------------------------------------------------------------
// FUNCTION DEFINITIONS
//----------------------------------------------------------------------------------------------

/**
 * @brief Opens and configures a serial port.
 *
 * This function opens the specified serial port and configures it with the provided baud rate.
 * If the port cannot be opened or configured, appropriate error messages are logged, and the 
 * function returns a failure status.
 *
 * @param serialName The file path of the serial port (e.g., "/dev/ttyS0").
 * @param baudRate A string representing the desired baud rate (e.g., "9600", "115200").
 * @return int32_t The file descriptor of the serial port on success, or -1 on failure.
 *
 * @note The function performs error checking at each step to ensure robust handling
 * of issues like non-existent ports, inaccessible ports, or invalid configurations.
 */
int32_t SerialPort::setupSerialPort(const std::string& serialName, const std::string& baudRate) {
    // Check if the serial port file exists
    if (access(serialName.c_str(), F_OK) == -1) {
        std::cerr << "Error: Serial port does not exist: " << serialName << std::endl;
        return -1;
    }

    // Open the serial port with read/write, no controlling terminal, and non-blocking mode
    int32_t serial_port = open(serialName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        std::cerr << "Error: Unable to open serial port: " << std::strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    // Get current terminal attributes
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << std::strerror(errno) << std::endl;
        close(serial_port);
        return -1;
    }

    // Convert baud rate string to speed_t
    speed_t baudRateSpeed;
    if (baudRate == "9600") baudRateSpeed = B9600;
    else if (baudRate == "19200") baudRateSpeed = B19200;
    else if (baudRate == "38400") baudRateSpeed = B38400;
    else if (baudRate == "57600") baudRateSpeed = B57600;
    else if (baudRate == "115200") baudRateSpeed = B115200;
    else baudRateSpeed = B9600;  // Default baud rate

    // Set input and output baud rates
    cfsetispeed(&tty, baudRateSpeed);
    cfsetospeed(&tty, baudRateSpeed);

    // Configure port settings
    tty.c_cflag &= ~PARENB;         // No parity
    tty.c_cflag &= ~CSTOPB;         // 1 stop bit
    tty.c_cflag &= ~CSIZE;          // Clear data size
    tty.c_cflag |= CS8;             // 8 data bits
    tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;  // Enable receiver and ignore modem control lines

    // Configure input modes
    tty.c_lflag &= ~ICANON;         // Raw input mode
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL);  // Disable all echo
    tty.c_lflag &= ~ISIG;           // Disable signal processing

    // Configure input flags
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
    tty.c_iflag &= ~(ICRNL | INLCR);         // Disable CR/NL translation

    // Configure output modes
    tty.c_oflag &= ~(OPOST | ONLCR);  // Raw output, disable NL to CR/NL

    // Configure read settings
    tty.c_cc[VMIN] = 1;   // Minimum bytes to read
    tty.c_cc[VTIME] = 0;  // Read timeout

    // Flush existing data and apply settings
    tcflush(serial_port, TCIFLUSH);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << std::strerror(errno) << std::endl;
        close(serial_port);
        return -1;
    }

    serial_port_fd = serial_port;
    return serial_port;
}

/**
 * @brief Closes the serial port to release resources.
 *
 * This function ensures that the file descriptor associated with the serial port
 * is properly closed when the port is no longer needed.
 */
void SerialPort::cleanupSerialPort() {
    close(serial_port_fd);
}

/**
 * @brief Reads data from the serial port.
 *
 * This function reads available data from the serial port into a buffer, using 
 * a robust mechanism to handle timeouts and partial reads. The data is then 
 * returned as a vector of bytes.
 *
 * @return std::vector<uint8_t> A vector containing the bytes read from the serial port.
 *
 * @note If no data is available or an error occurs, the function returns an empty vector.
 */
std::vector<uint8_t> SerialPort::readBytes() {
    std::vector<uint8_t> receivedData;

    // Use a more robust buffer management
    std::vector<uint8_t> buffer(256, 0);  // Pre-allocated buffer

    fd_set readfds;
    struct timeval tv;

    FD_ZERO(&readfds);
    FD_SET(serial_port_fd, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms timeout

    int32_t retval = select(serial_port_fd + 1, &readfds, NULL, NULL, &tv);
    if (retval == -1) {
        std::cerr << "Select error: " << strerror(errno) << std::endl;
        return receivedData;
    } else if (retval > 0) {
        ssize_t bytes_read = read(serial_port_fd, buffer.data(), buffer.size());
        if (bytes_read > 0) {
            receivedData.assign(buffer.begin(), buffer.begin() + bytes_read);
        } else if (bytes_read < 0) {
            std::cerr << "Read error: " << strerror(errno) << std::endl;
        }
    }

    return receivedData;
}

/**
 * @brief Sends data to the serial port.
 *
 * This function writes the provided data to the serial port. The data is sent
 * as a sequence of bytes.
 *
 * @param data A vector of bytes to be sent to the serial port.
 *
 * @note The function does not perform additional error handling for write operations.
 */
void SerialPort::sendData(const std::vector<uint8_t>& data) {
    write(serial_port_fd, data.data(), data.size());
}
