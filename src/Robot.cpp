#include "Robot.hpp"

#if _EXECUTION_ENVIRONMENT == 0
#include "MainMaze/robot/lib/extra/utils/Exceptions.hxx"
#else
#endif

SerialPort* Robot::serial_;
Compass* Robot::compass_;
Lasers* Robot::lasers_;
Gyro* Robot::gyro_;
Temp* Robot::temp_;

void Robot::Setup()
{
	serial_ = SerialPort::Instance();
	compass_ = Compass::Instance();
	lasers_ = Lasers::Instance();
	gyro_ = Gyro::Instance();
	temp_ = Temp::Instance();

	lasers_->Begin();
	gyro_->Begin(25);
	gyro_->Calibrate();
	temp_->Calibrate();

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

	compass_->GoTo(input_envelope->direction);

	Walls* walls = compass_->GetWalls();

	auto* output_envelope = new OutputEnvelope(walls, false, false);

	serial_->WriteEnvelope(output_envelope);

	return true;
}
