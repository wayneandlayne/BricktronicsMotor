// Bricktronics Example: MotorSingleBricktronicsMegashield
// http://www.wayneandlayne.com/bricktronics
//
// This example starts the motor at an intermediate speed,
// speeds it up to full speed, then does the same but in reverse.
//
// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot).
// It's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack, with a 2.1mm plug (center positive).
//
// Hardware used:
// * Wayne and Layne Bricktronics Megashield
//   https://store.wayneandlayne.com/products/bricktronics-megashield-kit.html
// * LEGO NXT or EV3 Motor
//
// Software libraries used:
// * Wayne and Layne BricktronicsMegashield library
//   https://github.com/wayneandlayne/BricktronicsMegashield
// * Wayne and Layne BricktronicsMotor library
//   https://github.com/wayneandlayne/BricktronicsMotor
//
// Written in 2016 by Matthew Beckler and Adam Wolf for Wayne and Layne, LLC
// To the extent possible under law, the author(s) have dedicated all
//   copyright and related and neighboring rights to this software to the
//   public domain worldwide. This software is distributed without any warranty.
// You should have received a copy of the CC0 Public Domain Dedication along
//   with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>. 


// Include the Bricktronics libraries
#include <BricktronicsMegashield.h>
#include <BricktronicsMotor.h>


// Select the desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
BricktronicsMotor m(BricktronicsMegashield::MOTOR_1);


void setup()
{
  // Be sure to set your serial console to 115200 baud
  Serial.begin(115200);

  // Initialize the motor connections
  m.begin();
}

void loop() 
{
  Serial.println("Going forward.");
  m.setFixedDrive(75);
  delay(1000);
  
  m.setFixedDrive(255);
  delay(1000);

  Serial.println("Going in reverse.");
  m.setFixedDrive(-75);
  delay(1000);
  
  m.setFixedDrive(-255);
  delay(1000);
}

