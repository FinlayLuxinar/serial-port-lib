#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <atomic>
#include <condition_variable>
#include <queue>
#include <map>
#include <termios.h>
#include <vector>
#include <string>

class SerialPort {
public:
    int32_t setupSerialPort(const std::string& serialName, const std::string& baudRate);
    void sendData(const std::vector<uint8_t>& data);
    bool waitForResponse(int32_t timeout_ms = 100);
    std::vector<uint8_t> readBytes();
    void cleanupSerialPort();

private:
    int32_t serial_port_fd;
    std::string accumulatedBuffer;
    uint8_t txDataBuffer[256];
    
    void processPacket(const std::string& packet);
    bool setChecksum(const std::string& received_checksum_str);
};

speed_t baudRateStringToSpeedT(const std::string& baudRate);

#endif // SERIALPORT_H
