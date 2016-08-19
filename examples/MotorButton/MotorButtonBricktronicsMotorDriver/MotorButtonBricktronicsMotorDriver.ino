// Bricktronics Example: MotorButtonBricktronicsMotorDriver
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
// * Wayne and Layne Bricktronics Motor Driver
//   https://store.wayneandlayne.com/products/bricktronics-motor-driver.html
// * Wayne and Layne Bricktronics Breakout Board
//   https://store.wayneandlayne.com/products/bricktronics-breakout-board.html
// * LEGO NXT or EV3 Motor
// * LEGO NXT Pushbutton Sensor
//
// Software libraries used:
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
#include <BricktronicsMotor.h>
#include <BricktronicsButton.h>


// For the motor, update the five pin assignments in the constructor below.
// The arguments are: enPin, dirPin, pwmPin, encoderPin1, encoderPin2
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (supports analogWrite)
//      Uno:       pins 3, 5, 6, 9, 10, and 11
//      Mega 2560: pins 2 to 13 and 44 to 46
// B. There are three ways to connect the encoder pins (labeled T1/T2 on the board).
// ** Best performance: Both signals are connected to true interrupt pins (listed below).
// ** Good performance: The FIRST signal (T1) is connected to an interrupt pin, the second signa is a regular pin. This is the mode used for the Bricktronics Shield/Megashield. For this mode it is CRITICAL that the true interrupt pin is used for T1 and not T2.
// ** Low performance: Both signals are connected to non-interrupt pins.
// Regardless of which performance mode used, you MUST list the pin T1 before T2 in
//   the constructor, otherwise the encoder will be connected backwards and the
//   PID algorithm will get all confused and freak out.
// Location of true interrupt pins:
//      Uno:       pins 2 and 3
//      Mega 2560: pins 2, 3, 21, 20, 19, and 18
//
// For the pushbutton, connect these pins on the Bricktronics Breakout board:
//  Pin 1 - Connect to Arduino input pin (any digital input pin)
//  Pin 2 - Connect to Ground
//  Pin 3 - Connect to Ground
//  Pin 4 - Connect to 5V
//  Pin 5 - No connection
//  Pin 6 - No connection
//
// The BricktronicsButton() argument is simply the Arduino pin where pin 1
// on the breakout board is connected.
//
BricktronicsMotor m(3, 4, 10, 2, 5);
BricktronicsButton b(7);



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

