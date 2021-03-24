#if _EXECUTION_ENVIRONMENT != 0
#include <Wire.h>
#include <Driver.hpp>
#include <Gyro.hpp>
#include <Lasers.hpp>
#include <SerialPort.hpp>
#include "Arduino.h"
#include <Communication/Directions.hxx>
#include <Communication/OutputMessage.hpp>

uint32_t start;

auto serial = SerialPort::Instance();
auto gyro = Gyro::Instance();
auto laser = Lasers::Instance();

void setup() {
    delay(500);
    Wire.begin();
    BusConnection::SetBus(&Wire);

    serial->Connect("", 9600);

    gyro->Begin(25);

    laser->Begin();
    laser->StartContinuous();

    start = millis();
}

void loop() {
    if (millis() - start > 500) {
        auto z = gyro->Yaw();
        auto st = millis();
        float f = laser->ReadF(), fl= laser->ReadFL(), fr = laser->ReadFR(), r = laser->ReadR();
        Serial.println(millis() - st);
        Serial.println("f:" + String(f) + " fl: " + String(fl) + " fr: " + String(fr) + " r: " + String(r));
        Serial.print("z: ");
        Serial.println(z);
        start = millis();
        OutputMessage msg((Walls) VDirection::LEFT, true, false, false, false);
        serial->Write(msg.toBinary(), 16);
    }
    gyro->Update();
    delay(25);
}
#endif
