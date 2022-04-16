#include "SerialPort.hpp"

HardwareSerial *SerialPort::serial;

void SerialPort::Handshake() const {
    uint8_t buffer[3];
    this->Read(buffer);
    this->Write(buffer, 3);
}


InputEnvelope *SerialPort::ReadEnvelope() const {
    uint8_t buffer[in_env_length];
    this->Read(buffer);
    char representation[in_env_length * 3];
    ToCharArray(buffer, representation, in_env_length);
    Logger::Info(kSerial, "reading: %s", representation);
    return InputEnvelope::FromBytes(buffer);
}

void SerialPort::WriteEnvelope(OutputEnvelope *envelope) const {
    uint8_t buffer[out_env_length];
    envelope->ToBytes(buffer);
    char representation[out_env_length * 3];
    ToCharArray(buffer, representation, out_env_length);
    Logger::Info(kSerial, "writing: %s", representation);
    this->Write(buffer, out_env_length);
}

uint8_t SerialPort::ReadHalfWayDrop() const {
    uint8_t buffer[3];
    this->Read(buffer);
    char representation[in_hw_length * 3];
    ToCharArray(buffer, representation, in_hw_length);
    Logger::Info(kSerial, "reading halfway: %s", representation);
    return buffer[1];
}

void SerialPort::WriteHalfWayPoint(uint8_t sides) const {
    uint8_t buffer[] = {0xfd, sides, 0xff};
    char representation[out_hw_length * 3];
    ToCharArray(buffer, representation, out_hw_length);
    Logger::Info(kSerial, "writing halfway: %s", representation);
    this->Write(buffer, 3);
}

void SerialPort::Read(uint8_t *buffer) const {
    uint8_t last_char, index = 0;
    do {
        last_char = ReadOneByte();
        buffer[index++] = last_char;
    } while (last_char != 0xff);
    if (buffer[0] == 0xfe) throw StopConnection();
}

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
        Logger::Error(kSerial, "error opening serial2 port %s", port_name);
        return;
    }
    Logger::Info(kSerial, "serial2 port %s open", port_name);

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

uint8_t SerialPort::ReadOneByte() const
{
    DWORD bytes_read{};
    uint8_t buffer[1];
    const unsigned int bytes_to_read = 1;
    memset(buffer, 0, 1);
    ReadFile(this->handler_, buffer, bytes_to_read, &bytes_read, nullptr);
    return buffer[0];
}

bool SerialPort::Write(uint8_t buffer[], const unsigned size) const
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
//    serial->setTx(PB3);
//    serial->setRx(PB2);
//    serial->begin(baud_rate);
}

uint8_t SerialPort::ReadOneByte() const {
    uint8_t buffer[1];
    while (serial->available() < 1);
    serial->readBytes(buffer, 1);
    return buffer[0];
}

bool SerialPort::Write(uint8_t *buffer, unsigned int size) const {
    return serial->write(buffer, size);
}

void SerialPort::Close() const {
    serial->end();
}

SerialPort::~SerialPort() {
    Close();
}

void SerialPort::SetBus(HardwareSerial *serial) {
    SerialPort::serial = serial;
}

#endif
