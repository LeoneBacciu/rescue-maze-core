#pragma once


#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "Core/Public/Windows/AllowWindowsPlatformTypes.h"
#include "MainMaze/robot/lib/extra/utils/Exceptions.hxx"
#include "MainMaze/robot/lib/Logger/Logger.hpp"
#include "Communication/Envelope.hpp"
#include "Windows.h"
#include <iostream>
#include "Core/Public/Windows/HideWindowsPlatformTypes.h"
#else

#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#include <utils/Exceptions.hxx>
#include <Communication/Envelope.hpp>
#include <Logger.hpp>

#endif


class SerialPort : public Singleton<SerialPort>, BusConnection {
    static const uint8_t in_env_length = 8;
    static const uint8_t out_env_length = 9;
    static const uint8_t in_hw_length = 3;
    static const uint8_t out_hw_length = 3;

public:

    void Connect(const char *port_name, int baud_rate);

    void Handshake() const;

    InputEnvelope *ReadEnvelope() const;

    void WriteEnvelope(OutputEnvelope *envelope) const;

    uint8_t ReadHalfWayDrop() const;

    void WriteHalfWayPoint(uint8_t sides) const;

    void Read(uint8_t *buffer) const;

    bool Write(uint8_t buffer[], unsigned size) const;

    void Close() const;

    static void SetBus(HardwareSerial *serial);

    ~SerialPort();

private:
    uint8_t ReadOneByte() const;

#if _EXECUTION_ENVIRONMENT == 0
    HANDLE handler_ = nullptr;
    bool connected_{};
    COMSTAT status_{};
    DWORD errors_{};
#else
    static HardwareSerial *serial;
#endif
};
