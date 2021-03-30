#include "Robot.hpp"


#if _EXECUTION_ENVIRONMENT == 0
#include "../lib/Compass/Compass.hpp"
#include "../lib/Serial/SerialPort.hpp"
#else
#include <Compass.hpp>
#include <Driver.hpp>
#include <Lasers.hpp>
#include <SerialPort.hpp>
#include <Temp.hpp>
#endif

void Robot::Setup()
{
	SerialPort* serial = SerialPort::Instance();
	Compass* compass = Compass::Instance();

	serial->Connect("COM4", 9600);

	serial->Handshake();

	InputEnvelope* envelope = serial->ReadEnvelope();
	
	UE_LOG(LogTemp, Warning, TEXT("Serial: %d"), envelope->drop);

	compass->GoTo(envelope->direction);
	
	serial->Close();
}

void Robot::Main()
{
}
