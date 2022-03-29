#include <Notification.hpp>
#include "Robot.hpp"

//SerialPort *Robot::serial_;
//Compass *Robot::compass_;
Lasers *Robot::lasers_;
Gyro *Robot::gyro_;
//Temp *Robot::temp_;
//Floor *Robot::floor_;
//Brick *Robot::brick_;
bool Robot::success_ = true;
InputEnvelope *Robot::last_envelope_ = new InputEnvelope(kRight, false, false);

HardwareSerial serial3(PB7, PB6);

void Robot::Setup() {
//    serial_ = SerialPort::Instance();
//    compass_ = Compass::Instance();
    lasers_ = Lasers::Instance();
    gyro_ = Gyro::Instance();
//    temp_ = Temp::Instance();
//    floor_ = Floor::Instance();
//    brick_ = Brick::Instance();

    Logger::AllowAllSources();
//	Logger::DenySource(kFloor);
    Logger::SetVerbosity(kVerbose);

    lasers_->Begin();
//    brick_->Begin();
    gyro_->Begin(100);
//    gyro_->Calibrate();
//    temp_->Calibrate();


    Logger::Verbose(kGeneric, "start");

    lasers_->StartContinuous();
    delay(100);

    while (1) {
        Logger::Warn(kGyro, "%.2f", gyro_->Yaw());
        Logger::Warn(kLasers, "R: %d, FR: %d, F: %d, FL: %d, L: %d", lasers_->ReadR(), lasers_->ReadFR(), lasers_->ReadF(), lasers_->ReadFL(), lasers_->ReadL());
        delay(250);
    }
//    brick_->Drop(-1);
//    Driver::Rotate(true);
//    Driver::Rotate(false);
//    Driver::Rotate(false);
//    Driver::Rotate(true);
//    Driver::Rotate(true);
//    Driver::Rotate(false);
//    Driver::Rotate(false);
//    Driver::Rotate(true);
//	Driver::SetSpeed(100, 100);
//    while (1);
//    Driver::Roztate(false);
//    Driver::Rotate(true);
//    Driver::Rotate(true);
//    Driver::Rotate(false);
//    Driver::Rotate(false);
//    Driver::Rotate(true);
//    brick_->Drop();
//    lasers_->StartContinuous();
//    whi        Serial2.println(lasers_->ComputeLateralDifference(500, 0));
//        delay(50);
//    }le (1) {
//        uint16_t c = lasers_->ReadF(), R = lasers_->ReadFR(), L = lasers_->ReadFL();
//        Serial2.printf("%d, %d, %d\n", L, c, R);
//        Serial2.println(lasers_->ComputeLateralDifference(500, 0));
//        delay(50);
//    }
//    uint32_t timer = millis();
//    for (int i = 0; i < 10000; ++i) {
//        if (millis() - 500 > timer) Serial2.println(gyro_->Yaw());
//        gyro_->Update();
//        delay(10);
//        Serial2.print("R: ");
//        Serial2.println(lasers_->ReadR());
//        Serial2.print("FR: ");
//        Serial2.println(lasers_->ReadFR());
//        Serial2.print("F: ");
//        Serial2.println(lasers_->ReadF());
//        Serial2.print("FL: ");
//        Serial2.println(lasers_->ReadFL());
//        Serial2.print("L: ");
//        Serial2.println(lasers_->ReadL());
//        Serial2.print("B: ");
//        Serial2.println(lasers_->ReadB());
//        delay(50);
//    }
//    while (1) {
//        Serial2.printf("x: %.2f, y: %.2f, z: %.2f\n", gyro_->Roll(), gyro_->Pitch(), gyro_->Yaw());
////        Serial2.print(" ");
////        Serial2.println(lasers_->ReadB());
////        Serial2.println(temp_->IsHot().left);
////        Serial2.println(temp_->IsHot().right);
//        delay(10);
//
//        }

    delay(100);
    Notification::Pulse(1);
//    Driver::Go();
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Rotate(true);
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Rotate(true);
//    Driver::Go();
//    Driver::Rotate(true);
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(true);
//    Driver::Go();
//    Driver::Rotate(true);
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(true);
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Go();
//    Driver::Rotate(false);
//    Driver::Go();
//    Driver::Go();
//    Driver::Go();
//    lasers_->StopContinuous();

//    compass_->GoTo(Direction::kTop, false, false);
//    while (1);
//    while (1) {
//        auto t = temp_->IsHot();
//        Serial2.printf("%d, %d", t.left, t.right);
//        delay(500);
//
//    }

//    serial_->Connect("COM4", 115200);
//
//    serial_->Handshake();
}

bool Robot::Main() {
//    Walls *walls = compass_->GetWalls();
//    const auto floor_type = floor_->Read();
//
//    auto *output_envelope = new OutputEnvelope(walls, !success_, floor_type == Floor::kCheckpoint);
//
//    serial_->WriteEnvelope(output_envelope);
//
//
//    InputEnvelope *input_envelope;
//    try {
//        input_envelope = serial_->ReadEnvelope();
//    }
//    catch (StopConnection &) {
//        Logger::Info(kCompass, "THE END");
//        serial_->Close();
//        return false;
//    }
//
//    if (input_envelope->drop != 0) compass_->Drop(input_envelope->drop);
//
//    success_ = compass_->GoTo(input_envelope->direction, last_envelope_->ignore, input_envelope->ignore);
//    last_envelope_ = input_envelope;
//    return true;
}
