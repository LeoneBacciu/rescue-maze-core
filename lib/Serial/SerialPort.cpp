#include "SerialPort.hpp"


#if _EXECUTION_ENVIRONMENT == 0
void SerialPort::Connect(const char* port_name, const int baud_rate)
{
	this->handler_ = CreateFileA(port_name,
	                             GENERIC_READ | GENERIC_WRITE,
	                             0,
	                             nullptr,
	                             OPEN_EXISTING,
	                             FILE_ATTRIBUTE_NORMAL,
	                             nullptr);
	if (this->handler_ == INVALID_HANDLE_VALUE)
	{
		std::cout << "Error opening serial port " << port_name << "\n";
		return;
	}

	DCB dcb_serial_params = {0};
	dcb_serial_params.DCBlength = sizeof(dcb_serial_params);

	GetCommState(this->handler_, &dcb_serial_params);

	dcb_serial_params.BaudRate = baud_rate; // Setting BaudRate = 9600
	dcb_serial_params.ByteSize = 8; // Setting ByteSize = 8
	dcb_serial_params.StopBits = ONESTOPBIT; // Setting StopBits = 1
	dcb_serial_params.Parity = NOPARITY; // Setting Parity = None
	dcb_serial_params.fDtrControl = DTR_CONTROL_ENABLE;

	PurgeComm(this->handler_, PURGE_RXCLEAR | PURGE_TXCLEAR);

	SetCommState(this->handler_, &dcb_serial_params);
}

int SerialPort::Read(char* buffer, const unsigned size) const
{
	DWORD bytes_read{};
	const unsigned int bytes_to_read = size;
	memset(buffer, 0, size);
	ReadFile(this->handler_, buffer, bytes_to_read, &bytes_read, nullptr);
	return bytes_read;
}

bool SerialPort::Write(char buffer[], const unsigned size) const
{
	DWORD bytes_written; // No of bytes written to the port

	return WriteFile(this->handler_, // Handle to the Serial port
	                 buffer, // Data to be written to the port
	                 size, //No of bytes to write
	                 &bytes_written, //Bytes written
	                 nullptr);
}

void SerialPort::Close() const
{
	CloseHandle(this->handler_); //Closing the Serial Port
}

SerialPort::~SerialPort()
{
	CloseHandle(this->handler_); //Closing the Serial Port
}
#else

void SerialPort::Connect(const char *port_name, int baud_rate) {
    Serial.begin(baud_rate);
}

int SerialPort::Read(char *buffer, unsigned int size) const {
    while (Serial.available() < size);
    return Serial.readBytes(buffer, size);
}

bool SerialPort::Write(char *buffer, unsigned int size) const {
    return Serial.write(buffer, size);
}

void SerialPort::Close() const {
    Serial.end();
}

SerialPort::~SerialPort() {
    Close();
}

#endif
