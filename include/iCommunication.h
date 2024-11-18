#ifndef ICOMMUNICATION_H
#define ICOMMUNICATION_H

#include <string>
#include <vector>

/**
 * @brief Interface for communication classes.
 * 
 * This interface defines the basic methods required for sending commands
 * through various communication methods (e.g., serial, network).
 */
class ICommunication {
public:
    /**
     * @brief Virtual destructor for interface.
     * 
     * Ensures proper cleanup of derived classes.
     */
    virtual ~ICommunication() = default;

    /**
     * @brief Sends a command to the communication medium.
     * 
     * @param command The command string to be sent.
     */
    virtual void sendData(const std::vector<uint8_t>& data) = 0;

};

#endif // ICOMMUNICATION_H
