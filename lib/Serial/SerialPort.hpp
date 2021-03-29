#pragma once


#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/Bus/BusConnection.hpp"
#include "MainMaze/robot/lib/extra/utils/Singleton.hxx"
#include "Core/Public/Windows/AllowWindowsPlatformTypes.h"
#include "Windows.h"
#include <iostream>
#include "Core/Public/Windows/HideWindowsPlatformTypes.h"
#else
#include <BusConnection.hpp>
#include <utils/Singleton.hxx>
#endif


class SerialPort : public Singleton<SerialPort>, BusConnection
{
public:
	void Connect(const char* port_name, int baud_rate);
	void Handshake() const;
	void Read(uint8_t* buffer) const;
	bool Write(uint8_t buffer[], unsigned size) const;
	void Close() const;
	~SerialPort();

private:
	uint8_t ReadOneByte() const;
#if _EXECUTION_ENVIRONMENT == 0
	HANDLE handler_ = nullptr;
	bool connected_{};
	COMSTAT status_{};
	DWORD errors_{};
#endif
};
