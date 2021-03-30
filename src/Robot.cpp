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

	InputEnvelope* input_envelope = serial->ReadEnvelope();
	
	UE_LOG(LogTemp, Warning, TEXT("Serial: %d"), input_envelope->drop);

	compass->GoTo(input_envelope->direction);

	Walls* walls = compass->GetWalls();

	OutputEnvelope* output_envelope = new OutputEnvelope(walls, false, false);

	serial->WriteEnvelope(output_envelope);
	
	serial->Close();
}

void Robot::Main()
{
}
