#include <cstdio>
#include <cstdint>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "hardware.h"

///
// H-bridge and PWM functions
///

HardwareMotor::HardwareMotor(int setPin1, int setPin2, int setPinPWM) :
   pin1(setPin1), pin2(setPin2), pinPWM(setPinPWM)
{
   // H-bridge control pins
   pinMode(pin1, OUTPUT);
   pinMode(pin2, OUTPUT);

   // PWM setup
   pinMode (pinPWM, PWM_OUTPUT);
   pwmSetMode(PWM_MODE_MS);
   pwmSetClock(16);
   pwmSetRange(100);
}

void HardwareMotor::turnOnDir1()
{
   digitalWrite(pin2, LOW);
   digitalWrite(pin1, HIGH);
}

void HardwareMotor::turnOnDir2()
{
   digitalWrite(pin1, LOW);
   digitalWrite(pin2, HIGH);
}

void HardwareMotor::turnOff()
{
   digitalWrite(pin1, LOW);
   digitalWrite(pin2, LOW);
}

void HardwareMotor::setPWM(unsigned short duty)
{
   pwmWrite(pinPWM, duty);
}

///
// AS5048A Rotary Sensor functions
///

#define BITCOUNT(x)     (((BX_(x)+(BX_(x)>>4)) & 0x0F0F0F0F) % 255)
#define BX_(x)          ((x) - (((x)>>1)&0x77777777) \
                             - (((x)>>2)&0x33333333) \
                             - (((x)>>3)&0x11111111))

// SPI commands

#define FLAG_READ 0x4000
#define CMD_ANGLEDATA 0x3fff
#define CMD_MAGDATA 0x3ffe
#define CMD_DIAGDATA 0x3ffd
#define CMD_NOOP 0x0000

uint16_t sendReceive(uint16_t command, bool verbose = false)
{
   if (BITCOUNT(command) % 2)
      command |= 0x8000;

   unsigned char data[2];
   data[0] = command >> 8;
   data[1] = command & 0xff;

   if (verbose)
      printf("sending %02x%02x   ", data[0], data[1]);

   wiringPiSPIDataRW(0, data, 2);

   if (verbose)
   printf("received %02x%02x\n", data[0], data[1]);

   return (data[0] << 8) + data[1];
}


RawAngle HardwareSensor::getRawAngle()
{
   // Send a SPI request for angle data, ignoring the result as it belongs
   // to the previously issued command.
   sendReceive(CMD_ANGLEDATA | FLAG_READ);

   // Flush the command with a NOOP and record the reply.
   uint16_t angledata = sendReceive(CMD_NOOP);

   return (float)(angledata & 0x3fff) * 360.0f / 0x3fff;
}
