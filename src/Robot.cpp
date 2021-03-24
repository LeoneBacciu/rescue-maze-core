#include "Robot.hpp"


#if _EXECUTION_ENVIRONMENT == 0
#include "../lib/Compass/Compass.hpp"
#include "../lib/Driver/Driver.hpp"
#include "../lib/Lasers/Lasers.hpp"
#include "../lib/Serial/SerialPort.hpp"
#include "../lib/Temp/Temp.hpp"
#else
#include <Compass.hpp>
#include <Driver.hpp>
#include <Lasers.hpp>
#include <SerialPort.hpp>
#include <Temp.hpp>
#endif

void Robot::Setup()
{
	Lasers* lasers = Lasers::Instance();
	Temp* temps = Temp::Instance();
	Driver* driver = Driver::Instance();
	SerialPort* serial = SerialPort::Instance();
	Compass* compass = Compass::Instance();

	// for (int i = 0; i < 10000; ++i)
	// {
	// 	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Emerald,FString::Printf(TEXT("Angle: %f"), FMath::RadiansToDegrees(lasers->computeFrontAngle())));
	// 	FPlatformProcess::Sleep(0.2);
	// }

	// Walls walls = std::make_tuple(0x00, 0x01, 0x00, 0x01);
	// Walls walls{0x00, 0x01, 0x01, 0x01};
	// OutputMessage message(walls, true, false, true, false);
	//
	// FPlatformProcess::Sleep(0.1);
	//
	// GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                  FString::Printf(
	//                                      TEXT("Serial: %d"),
	//                                      serial->write(message.toBinary(), Communication::MESSAGE_LENGTH)));
	// serial->close();
	compass->GoTo(kTop);
	compass->GoTo(kTop);
	compass->GoTo(kTop);
	compass->GoTo(kLeft);
	compass->GoTo(kTop);
	compass->GoTo(kRight);
	compass->GoTo(kRight);
	compass->GoTo(kTop);
	compass->GoTo(kTop);
	compass->GoTo(kRight);
	compass->GoTo(kRight);
	compass->GoTo(kBottom);
	compass->GoTo(kBottom);
	compass->GoTo(kBottom);
	compass->GoTo(kLeft);
	compass->GoTo(kBottom);
	compass->GoTo(kBottom);
	compass->GoTo(kBottom);
	compass->GoTo(kRight);
	compass->GoTo(kTop);
	compass->GoTo(kTop);
	compass->GoTo(kRight);
	compass->GoTo(kTop);
	compass->GoTo(kLeft);
	compass->GoTo(kTop);
	compass->GoTo(kLeft);
	compass->GoTo(kLeft);
	compass->GoTo(kLeft);
	compass->GoTo(kLeft);
	compass->GoTo(kBottom);
	compass->GoTo(kBottom);
	
	// if (GEngine)
	// {
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance F: %f"), lasers->readF()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance FR: %f"), lasers->readFR()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance FL: %f"), lasers->readFL()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance R: %f"), lasers->readR()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance L: %f"), lasers->readL()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance B: %f"), lasers->readB()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red,
	//                                     FString::Printf(TEXT("Temp R: %f"), temps->read().right));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red,
	//                                     FString::Printf(TEXT("Temp L: %f"), temps->read().left));
	// }

	// if (GEngine)
	// {
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance F: %f"), lasers->readF()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance FR: %f"), lasers->readFR()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance FL: %f"), lasers->readFL()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance R: %f"), lasers->readR()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance L: %f"), lasers->readL()));
	//     GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow,
	//                                      FString::Printf(TEXT("Distance B: %f"), lasers->readB()));
	// }
	// driver->go();
	// driver->rotate(true);
	// driver->go();
	// driver->go();
	// driver->rotate(false);
	// driver->go();
	// driver->rotate(true);
	// driver->go();
}

void Robot::Main()
{
}
