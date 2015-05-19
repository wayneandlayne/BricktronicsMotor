// Bricktronics Example: MotorButton
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor and a Pushbutton sensor.
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


// Include the Bricktronics Motor library and helper libraries
// Helper libraries can be download from:
// https://www.pjrc.com/teensy/td_libs_Encoder.html
// https://github.com/br3ttb/Arduino-PID-Library/
//	Be sure to rename unzipped folder PID_v1
#include <Encoder.h>
#include <PID_v1.h>
#include <BricktronicsMotor.h>
// Include the Bricktronics Button libraries
#include <BricktronicsButton.h>


// This example can be run in three different ways. Pick one, and un-comment
// the code lines corresponding to your chosen method. Comment-out the lines
// for the other methods that you aren't using.

// 1. With a Bricktronics Shield - Include these lines and be sure to
// call BricktronicsShield::begin() in the setup() function below.
// You also need to install the Adafruit MCP23017 library:
//	https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
// Select the motor port (MOTOR_1 or MOTOR_2) and sensor port
// (SENSOR_1 through SENSOR_4) in the constructors below.
// If your chosen sensor port has jumpers (ports 3 and 4), connect pins 2-3 and 4-5.
//
//#include <Wire.h>
//#include <Adafruit_MCP23017.h>
//#include <BricktronicsShield.h>
//BricktronicsMotor m(BricktronicsShield::MOTOR_1);
//BricktronicsButton b(BricktronicsShield::SENSOR_1);

// 2. With a Bricktronics Megashield - Include these lines but do not
// call BricktronicsShield::begin() in the setup() function below.
// Select the desired motor port (MOTOR_1 through MOTOR_6) and sensor port
// (SENSOR_1 through SENSOR_4) in the constructors below.
// Connect pins 2-3 and 4-5 on the chosen sensor port.
//
// #include <BricktronicsMegashield.h>
//BricktronicsMotor m(BricktronicsMegashield::MOTOR_1);
//BricktronicsButton b(BricktronicsMegashield::SENSOR_1);

// 3. With a Bricktronics Motor Driver and Breakout board - No additional includes needed,
// just update the pin assignments in the Motor and Button constructors below.
//
// The BricktronicsMotor() arguments are: enPin, dirPin, pwmPin, tachPinA, tachPinB
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (analogWrite)
// Uno:       pins 3, 5, 6, 9, 10, and 11
// Mega 2560: pins 2-13 and 44-46
// B. At least one of tachPinA/B needs to be an actual interrupt pin (not just
// a "pin change interrupt" pin).
// Uno:       pins 2 and 3
// Mega 2560: 2, 3, 21, 20, 19, and 18
//
// The BricktronicsButton() argument is simply the pin the button is connected to,
// that is, wherever pin 1 of the Breakout board is connected (also connect the grounds).
// No worries about PWM or interrupt pins here.
//
//BricktronicsMotor m(3, 4, 10, 2, 5);
//BricktronicsButton b(7);


void setup()
{
  // Only call this if you are using a Bricktronics Shield,
  // otherwise leave it commented-out.
  //BricktronicsShield::begin();

  // Initialize the motor and button connections
  m.begin();
  b.begin();
}

int theSpeed = 150;

void loop()
{
  m.rawSetSpeed(theSpeed);
  
  // Wait until the button is pressed
  while (b.isReleased())
  {
    // Nothing to do here
  }
  // To get here, the button was pushed!

  // Stop the motor while the button is held down
  m.stop();

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

