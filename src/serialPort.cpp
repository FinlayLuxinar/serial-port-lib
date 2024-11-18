//----------------------------------------------------------------------------------------------
// INCLUDE SELF
//----------------------------------------------------------------------------------------------
#include "serialPort.h"
#include "iCommunication.h"
#include "iParser.h"
//----------------------------------------------------------------------------------------------
// STANDARD INCLUDES
//----------------------------------------------------------------------------------------------
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <thread>

//----------------------------------------------------------------------------------------------
// PRIVATE DEFINES
//----------------------------------------------------------------------------------------------
// #define BAUD_RATE B9600
#define SHOWEXTENDEDSTATUS true

//----------------------------------------------------------------------------------------------
// IMPLEMENTATION
//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------------------------------
/**
 * 
 * @brief Constructor for the SerialPort class.
 * 
 * This constructor initializes the SerialPort object with the GPIO, ReplyParser, and IPublisher
 * instances passed as references.
 * 
 */
// SerialPort::SerialPort(GPIO& gpio_ref, ReplyParser& rp)
//     : gpio(gpio_ref), replyParser(rp) {} // Constructor initialization
SerialPort::SerialPort(IParser* parser) 
    : replyParser(parser) {
    if (!parser) {
        throw std::invalid_argument("Parser cannot be null");
    }
} 


// Function to open the serial port
/**
 * 
 * @brief Opens the serial port with the specified name.
 * 
 * This function opens the serial port with the specified name and returns the file descriptor
 * for the port. If the serial port does not exist, the function prints an error message and
 * exits the program with a failure status.
 * 
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
// Function to clean up the serial port
/**
    * 
    * @brief Cleans up the serial port by closing the file descriptor.
    * 
    * This function closes the file descriptor for the serial port to clean up the resources
    * used by the port.
    * 
    */
void SerialPort::cleanupSerialPort() {
    close(serial_port_fd);
}



// Function to log an error message
/**
 * 
 * @brief Logs an error message to the logger instance.
 * 
 * This function logs an error message to the logger instance, which writes the message to the log
 * file along with a timestamp.
 * 
 */

// Function to receive data from the serial port
/**
 * 
 * @brief Receives data from the serial port and processes it.
 * 
 * This function receives data from the serial port and processes it by extracting complete
 * packets, parsing the packets, and calling the corresponding reply parser functions.
 * 
 */
// void SerialPort::receiveData(int32_t serial_port, GPIO& gpio_ref) {
void SerialPort::receiveData(int32_t serial_port) {
    
    char buffer[256];
    fd_set readfds;
    struct timeval tv;

    FD_ZERO(&readfds);
    FD_SET(serial_port, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    int32_t retval = select(serial_port + 1, &readfds, NULL, NULL, &tv);
    if (retval == -1) {
        std::cerr << "DEBUG: Select error" << std::endl;
    } else if (retval) {
        ssize_t bytes_read = read(serial_port, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            rxBuffer.append(buffer, bytes_read);

            size_t pos;
            while ((pos = rxBuffer.find("\r\n")) != std::string::npos) {
                std::string packet = rxBuffer.substr(0, pos);
                rxBuffer.erase(0, pos + 2);

                lastResponse = packet;
                responseReceived = true;
                //gpio_ref.setValue(false);
                replyParser->parseReply(packet);
            }
        }
    }
}

// Function to send a command to the serial port
/**
 * 
 * @brief Sends a command to the serial port.
 * 
 * This function sends a command to the serial port by adding the command to the command queue
 * and notifying the thread to process the command.
 * 
 */
void SerialPort::sendData(const std::vector<uint8_t>& data) {
        write(serial_port_fd, data.data(), data.size());
    }

// Function to wait for a response to a command

bool SerialPort::waitForResponse(int32_t timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    while (!responseReceived) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() > timeout_ms) {
            std::cout << "DEBUG: Timeout waiting for response to command: " << currentCommand << std::endl;
            return false;
        }
    }

    responseReceived = false;
    return true;
}
