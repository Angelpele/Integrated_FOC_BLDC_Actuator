//===================================================================================================//
//                                   Cross-platform serial port                                      //
//===================================================================================================//
// Minimal, dependency-free serial port wrapper. Blocking reads with a short
// timeout so the reader thread can poll a "stop" flag cheaply. Windows uses
// the Win32 COM API, everything else (Linux/macOS) uses POSIX termios.
#pragma once

#include <string>
#include <vector>
#include <cstdint>

struct PortInfo {
    std::string device;       // e.g. "COM5" or "/dev/ttyUSB0"
    std::string description;  // best-effort friendly name
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Opens the port at the given baud rate (8N1, no flow control).
    // Returns true on success; on failure, err contains a human message.
    bool open(const std::string& device, int baud, std::string& err);
    void close();
    bool isOpen() const { return open_; }

    // Reads whatever is available (up to maxBytes), blocking up to ~100ms.
    // Returns number of bytes read (0 on timeout, <0 on hard error).
    int readSome(uint8_t* buf, int maxBytes);

    // Writes the full buffer. Returns true on success.
    bool write(const std::string& data);

    // Enumerates serial ports currently visible to the OS.
    static std::vector<PortInfo> listPorts();

private:
    bool open_ = false;
#if defined(_WIN32)
    void* handle_ = nullptr; // HANDLE, kept void* to avoid pulling windows.h into every TU
#else
    int fd_ = -1;
#endif
};
