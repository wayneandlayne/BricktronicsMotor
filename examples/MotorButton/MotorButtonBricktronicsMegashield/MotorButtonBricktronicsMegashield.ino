// Bricktronics Example: MotorButtonBricktronicsMegashield
// http://www.wayneandlayne.com/bricktronics
//
// This example starts the motor at full speed, then waits for
// the button to be pressed and released, then reverses direction.
// It does this forever! (or until you turn off the power,
// unplug stuff, or reprogram the Arduino.)
//
// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot)
// it's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack (2.1mm plug, center positive).
//
// Hardware used:
// * Wayne and Layne Bricktronics Megashield
//   https://store.wayneandlayne.com/products/bricktronics-megashield-kit.html
// * LEGO NXT or EV3 Motor
// * LEGO NXT Pushbutton Sensor
//
// Software libraries used:
// * Wayne and Layne BricktronicsMegashield library
//   https://github.com/wayneandlayne/BricktronicsMegashield
// * Wayne and Layne BricktronicsMotor library
//   https://github.com/wayneandlayne/BricktronicsMotor
// * Wayne and Layne BricktronicsButton library
//   https://github.com/wayneandlayne/BricktronicsButton
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
#include <BricktronicsButton.h>


// Select the desired motor port (MOTOR_1 through MOTOR_6) and sensor port
// (SENSOR_1 through SENSOR_4) in the constructors below.
// Connect pins 2-3 and 4-5 on the chosen sensor port.
BricktronicsMotor m(BricktronicsMegashield::MOTOR_1);
BricktronicsButton b(BricktronicsMegashield::SENSOR_1);


void setup()
{
  // Initialize the motor and button connections
  m.begin();
  b.begin();
}

int theSpeed = 150;

void loop()
{
  m.setFixedDrive(theSpeed);
  
  // Wait until the button is pressed
  while (b.isReleased())
  {
    // Nothing to do here
  }
  // To get here, the button was pushed!

  // While the button is held down, turn on the dynamic brake
  m.brake();

  // In order to debounce the button, we wait a little bit here
  delay(100);
  
  // Wait until the button is released
  while (b.isPressed())
  {
    // Nothing to do here
  }

  // In order to debounce the button, we wait a little bit here
  delay(100);

  // Reverse direction
  theSpeed *= -1;
}

