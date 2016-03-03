/**
 * PWM Robot
 *
 * Test sketch for pwm robot
 *
 *	author: João Campos
 *
 *
 *
 */

// include PWM Robot library for robot
#include "BluetoothRobot.h"

// --------------------------------------------------------------------------- Motors
const int motorLeft[] = {10, 11};
const int motorRight[] = {3, 9};
const int RX = 12;
const int TX = 13;

BluetoothRobot InovaHackerRobot(motorLeft, motorRight, RX, TX);

// --------------------------------------------------------------------------- Setup
void setup() {
//Serial.begin(9600);

//begin robot
InovaHackerRobot.setup();
}

// --------------------------------------------------------------------------- Loop

void loop() {

  InovaHackerRobot.readBluetooth();
}
