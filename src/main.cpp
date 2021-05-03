#if _EXECUTION_ENVIRONMENT != 0

#include <Wire.h>
#include "Arduino.h"

#define lol
#ifdef lol

#include "Robot.hpp"


void setup() {
    Serial2.begin(115200);
    Serial1.setTx(PB6);
    Serial1.setRx(PB7);
    Serial1.begin(115200);
    Serial2.println("STARTING");
    delay(125);

    while (!Serial2.available());

    pinMode(PA10, OUTPUT);
    pinMode(PB12, OUTPUT);
    pinMode(PB13, OUTPUT);
    pinMode(PA9, OUTPUT);
    pinMode(PB14, OUTPUT);
    pinMode(PB15, OUTPUT);

    Wire.begin((uint8_t) PB9, (uint8_t) PB8);

    BusConnection::SetBus(&Wire);

    for (int i = 0; i < 8; ++i) {
        Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
        Wire.write(1 << i);          // send byte to select bus
        Wire.endTransmission();
        int nDevices = 0;

        Serial2.print("Scanning bus ");
        Serial2.print(i);
        Serial2.println("...");

        for (byte address = 1; address < 127; ++address) {
            Wire.beginTransmission(address);
            byte error = Wire.endTransmission();

            if (error == 0) {
                Serial2.print("I2C device found at address 0x");
                if (address < 16) {
                    Serial2.print("0");
                }
                Serial2.print(address, HEX);
                Serial2.println("  !");

                ++nDevices;
            } else if (error == 4) {
                Serial2.print("Unknown error at address 0x");
                if (address < 16) {
                    Serial2.print("0");
                }
                Serial2.println(address, HEX);
            }
        }
        if (nDevices == 0) {
            Serial2.println("No I2C devices found\n");
        } else {
            Serial2.println("done\n");
        }
    }

    Serial2.println("SETUP");
    Robot::Setup();
//    while (1);
}

void loop() {
    if (!Robot::Main()) return;
}


#else



void setup() {

//    Wire.begin();
//    Wire.setClock(400000);
//    Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
//    Wire.write(1 << 6);          // send byte to select bus
//    Wire.endTransmission();

    Serial2.begin(115200);

}

void loop() {
    uint16_t (*flash_size) = (uint16_t*)(0x1ffff7e0);
    Serial2.print("ciao ");
    Serial2.println(*flash_size);
    delay(100);
}

#endif

#endif
