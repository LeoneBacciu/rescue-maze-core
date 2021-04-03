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
Floor* Robot::floor_;

void Robot::Setup()
{
	serial_ = SerialPort::Instance();
	compass_ = Compass::Instance();
	lasers_ = Lasers::Instance();
	gyro_ = Gyro::Instance();
	temp_ = Temp::Instance();
	floor_ = Floor::Instance();

	lasers_->Begin();
	gyro_->Begin(25);
	gyro_->Calibrate();
	temp_->Calibrate();

	serial_->Connect("COM4", 9600);

	serial_->Handshake();
}

bool Robot::Main()
{
	Walls* walls = compass_->GetWalls();
	const auto floor_type = floor_->Read();

	auto* output_envelope = new OutputEnvelope(walls, floor_type == Floor::kBlack, floor_type == Floor::kCheckpoint);

	serial_->WriteEnvelope(output_envelope);

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
	return true;
}
