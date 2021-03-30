#include "Robot.hpp"

#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/extra/utils/Exceptions.hxx"
#else
#include <Compass.hpp>
#include <Driver.hpp>
#include <Lasers.hpp>
#include <SerialPort.hpp>
#include <Temp.hpp>
#endif

SerialPort* Robot::serial_;
Compass* Robot::compass_;

void Robot::Setup()
{
	serial_ = SerialPort::Instance();
	compass_ = Compass::Instance();

	serial_->Connect("COM4", 9600);

	serial_->Handshake();
}

bool Robot::Main()
{
	InputEnvelope* input_envelope;
	try
	{
		input_envelope = serial_->ReadEnvelope();
	}
	catch (StopConnection&)
	{
		serial_->Close();
		return false;
	}

	UE_LOG(LogTemp, Warning, TEXT("Serial: %d"), input_envelope->drop);

	compass_->GoTo(input_envelope->direction);

	Walls* walls = compass_->GetWalls();

	OutputEnvelope* output_envelope = new OutputEnvelope(walls, false, false);

	serial_->WriteEnvelope(output_envelope);

	return true;
}
