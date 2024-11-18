#ifndef SERIALPORT_H
#define SERIALPORT_H
//---------------------------------------------------------------------
// PROJECT INCLUDES
//---------------------------------------------------------------------

//#include "gpio.h"
#include "iCommunication.h"
#include "iParser.h"
//---------------------------------------------------------------------
// STANDARD INCLUDES
//---------------------------------------------------------------------
#include <atomic>
#include <condition_variable>
#include <queue>
#include <map>
#include <termios.h>

//---------------------------------------------------------------------
// FORWARD DECLARATIONS
//---------------------------------------------------------------------

/**
 * @class SerialPort
 * @brief Handles serial port communication.
 * 
 * This class manages the opening, configuring, and communication
 * through a serial port, including sending commands and processing
 * incoming data.
 */
class SerialPort : public ICommunication {

//---------------------------------------------------------------------
// PUBLIC TYPES
//---------------------------------------------------------------------
public:
    int32_t setupSerialPort(const std::string& serialName, const std::string& baudRate);
    /**
     * @brief Constructs a SerialPort object.
     * @param gpio_ref Reference to GPIO object.
     * @param rp Reference to ReplyParser object.
     * @param pub Reference to IPublisher object.
     */
    //SerialPort(GPIO& gpio_ref, ReplyParser& rp); // Constructor declaration
    explicit SerialPort(IParser* parser); // Updated constructor

    // Buffers and flags for managing data and communication
    std::string lastResponse;               ///< Last received response
    std::string currentCommand;             ///< Current command being processed
    std::condition_variable pauseCV;       ///< Condition variable for pausing the thread
    std::atomic<bool> packetReceived{false}; ///< Flag indicating if a packet was received
    std::atomic<bool> threadRunning{true};  ///< Flag indicating if the thread is running
    std::atomic<bool> threadPaused{false};  ///< Flag indicating if the thread is paused
    std::mutex pauseMutex;                  ///< Mutex for pausing the thread
    std::queue<std::string> commandQueue;  ///< Queue for storing commands to be sent
    std::atomic<bool> responseReceived{false}; ///< Flag indicating if a response was received
    std::mutex queueMutex;                  ///< Mutex for accessing the command queue
    std::condition_variable queueCV;       ///< Condition variable for command queue operations

    // Typedef for command handler functions
    typedef void (SerialPort::*SerialFunc)();
    //GPIO& getGPIO() { return gpio; } ///< Get reference to GPIO object

    // Map associating command IDs with corresponding handler functions
    std::map<uint16_t, SerialPort::SerialFunc> serialFuncMap;

//---------------------------------------------------------------------
// PUBLIC METHODS
//---------------------------------------------------------------------

    /**
     * @brief Creates and configures the serial port.
     * @param serialName Name of the serial port.
     * @param baudRate Baud rate for communication.
     * @return File descriptor of the opened serial port.
     */
    int32_t createAndConfigureSerialPort(const std::string& serialName, const std::string& baudRate);
    
    /**
     * @brief Receives data from the serial port.
     * @param serial_port File descriptor of the serial port.
     * @param gpio_ref Reference to GPIO object for signaling.
     */
    // void receiveData(int32_t serial_port, GPIO& gpio_ref);
    void receiveData(int32_t serial_port);
    
    /**
     * @brief Sends a command to the serial port.
     * @param command Command string to send.
     */
    void sendData(const std::vector<uint8_t>& data);
    
    /**
     * @brief Waits for a response within the specified timeout.
     * @param timeout_ms Timeout in milliseconds (default is 100 ms).
     * @return True if response received, false otherwise.
     */
    bool waitForResponse(int32_t timeout_ms = 100);

//---------------------------------------------------------------------
// PRIVATE MEMBERS
//---------------------------------------------------------------------
private:
    std::string rxBuffer;                  ///< Buffer for receiving data
    uint8_t txDataBuffer[256];             ///< Buffer for transmitting data
    std::string accumulatedBuffer;        ///< Buffer for accumulated data
    int32_t serial_port_fd;                 ///< File descriptor for the serial port
    IParser* replyParser; // Change to IParser pointer
    //GPIO& gpio;                             ///< Reference to GPIO object

//---------------------------------------------------------------------
// PRIVATE METHODS
//---------------------------------------------------------------------
    /**
     * @brief Processes a received packet.
     * @param packet The packet to process.
     */
    void processPacket(const std::string& packet);
    
    /**
     * @brief Validates the received checksum against the calculated checksum.
     * @param received_checksum_str The received checksum as a string.
     * @return True if checksums match, false otherwise.
     */
    bool setChecksum(const std::string& received_checksum_str);
    
    /**
     * @brief Cleans up the serial port.
     */
    void cleanupSerialPort();
    
    /**
     * @brief Opens the serial port.
     * @param serialName Name of the serial port.
     * @return File descriptor of the opened serial port.
     */
    int32_t openSerialPort(const std::string& serialName);
    
    /**
     * @brief Configures the serial port settings.
     * @param serial_port File descriptor of the serial port.
     * @param baudRate Baud rate for communication.
     * @return 0 on success, or an error code on failure.
     */
    int32_t configureSerialPort(int32_t serial_port, const std::string& baudRate);
    
    /**
     * @brief Processes a command based on the command ID.
     * @param commandID The ID of the command to process.
     */
    void processCommand(uint16_t commandID);
    
    /**
     * @brief Logs an error message.
     * @param errorMessage The error message to log.
     */
    void logError(const std::string& errorMessage);  
};

/**
 * @brief Converts a baud rate string to speed_t value.
 * @param baudRate Baud rate as a string (e.g., "9600").
 * @return Corresponding speed_t value.
 */
speed_t baudRateStringToSpeedT(const std::string& baudRate);
//---------------------------------------------------------------------
// EXTERNAL FUNCTIONS
//---------------------------------------------------------------------
// External functions for opening and configuring the serial port

#endif // SERIALPORT_H
