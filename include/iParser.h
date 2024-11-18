#ifndef IPARSER_H
#define IPARSER_H

#include <string>

/**
 * @brief Interface for parsing serial communication replies
 * 
 * This interface defines the contract for classes that handle parsing of
 * serial communication packets. By using this interface, clients can work with
 * different parser implementations without being tightly coupled to them.
 */
class IParser {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes
     */
    virtual ~IParser() = default;

    /**
     * @brief Parses a reply packet from serial communication
     * 
     * @param packet The complete packet string to parse
     */
    virtual void parseReply(const std::string& packet) = 0;
};

#endif // IPARSER_H