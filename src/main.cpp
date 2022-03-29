#if _EXECUTION_ENVIRONMENT != 0

#include <Wire.h>
#include "Arduino.h"
#include "BusConnection.hpp"

#define lol
#ifdef lol

#include "Robot.hpp"

TwoWire wire(PB9, PB8);
HardwareSerial serial(PB7, PB6);

void setup() {

    digitalWrite(PA15, LOW);

    serial.begin(115200);
    serial.println("STARTING");
    delay(125);


    pinMode(PA10, OUTPUT);
    pinMode(PB12, OUTPUT);
    pinMode(PB13, OUTPUT);
    pinMode(PA9, OUTPUT);
    pinMode(PB14, OUTPUT);
    pinMode(PB15, OUTPUT);
    pinMode(PA8, OUTPUT);


    wire.begin();
    wire.setClock(400000);


    BusConnection::SetBus(&wire);
    Logger::SetBus(&serial);



    for (int i = 0; i < 8; ++i) {
        wire.beginTransmission(0x70);  // TCA9548A address is 0x70
        wire.write(1 << i);          // send byte to select bus
        wire.endTransmission();
        int nDevices = 0;

        serial.print("Scanning bus ");
        serial.print(i);
        serial.println("...");

        for (byte address = 1; address < 127; ++address) {
            wire.beginTransmission(address);
            byte error = wire.endTransmission();

            if (error == 0) {
                serial.print("I2C device found at address 0x");
                if (address < 16) {
                    serial.print("0");
                }
                serial.print(address, HEX);
                serial.println("  !");

                ++nDevices;
            } else if (error == 4) {
                serial.print("Unknown error at address 0x");
                if (address < 16) {
                    serial.print("0");
                }
                serial.println(address, HEX);
            }
        }
        if (nDevices == 0) {
            serial.println("No I2C devices found\n");
        } else {
            serial.println("done\n");
        }
    }

    while (!serial.available());

    serial.println("SETUP");
    Robot::Setup();
}

void loop() {
//    if (!Robot::Main()) while (1);
}


#else


int i = 0;

void setup() {


//    Wire.begin();
    //Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
    //1Wire.write(1 << 4);          // send byte to select bus
    //w/ire.endTransmission();

    serial.begin(115200);
    serial.println("\nI2C Scanner");
    Wire.begin((uint8_t) PB9, (uint8_t) PB8);
    Wire.setClock(400000);

}

void loop() {
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << i);          // send byte to select bus
  Wire.endTransmission();
  int nDevices = 0;

  serial.print("Scanning bus ");
  serial.print(i);
  serial.println("...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      serial.print("I2C device found at address 0x");
      if (address < 16) {
        serial.print("0");
      }
      serial.print(address, HEX);
      serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      serial.print("Unknown error at address 0x");
      if (address < 16) {
        serial.print("0");
      }
      serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    serial.println("No I2C devices found\n");
  } else {
    serial.println("done\n");
  }

  if (i<7) i++;
  else i = 0;
  delay(1000); // Wait 5 seconds for next scan
}

#endif

#endif
