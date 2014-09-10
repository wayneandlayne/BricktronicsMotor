// Bricktronics Example: MotorButton
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor and a Pushbutton sensor.

// This example starts the motor at full speed, then waits for
// the button to be pressed and released, then reverses direction.
// It does this forever! (or until you turn off the power,
// unplug stuff, or reprogram the Arduino.)

// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot)
// it's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack (2.1mm plug, center positive).


// Include the Bricktronics Motor and Button libraries
#include <Motor.h>
#include <Button.h>


// This example can be run in three different ways. Pick one, and un-comment
// the code lines corresponding to your chosen method. Comment-out the lines
// for the other methods that you aren't using.

// 1. With a Bricktronics Shield - Include these lines and be sure to
// call Bricktronics::begin() in the setup() function below.
// Select the motor port (BS_MOTOR_1 or BS_MOTOR_2) and sensor port
// (BS_SENSOR_1 through BS_SENSOR_4) in the constructors below.
// If your chosen sensor port has jumpers (ports 3 and 4), connect pins 2-3 and 4-5.
//
//#include <Wire.h>
//#include <Bricktronics2.h>
//Motor m = Motor(Bricktronics::BS_MOTOR_1);
//Button b = Button(Bricktronics::BS_SENSOR_1);

// 2. With a Bricktronics Megashield - Include these lines but do not
// call Bricktronics::begin() in the setup() function below.
// Select the desired motor port (BMS_MOTOR_1 through BMS_MOTOR_6) and sensor port
// (BMS_SENSOR_1 through BMS_SENSOR_4) in the constructors below.
// Connect pins 2-3 and 4-5 on the chosen sensor port.
// #include <Bricktronics2.h>
//
//Motor m = Motor(Bricktronics::BMS_MOTOR_1);
//Button b = Button(Bricktronics::BMS_SENSOR_1);

// 3. With a Bricktronics Motor Driver and Breakout board - No additional includes needed,
// just update the pin assignments in the Motor and Button constructors below.
//
// The Motor() arguments are: enPin, dirPin, pwmPin, tachPinA, tachPinB
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (analogWrite)
// Uno:       pins 3, 5, 6, 9, 10, and 11
// Mega 2560: pins 2 to 13 and 44 to 46
// B. At least one of tachPinA/B needs to be an actual interrupt pin (not just
// a "pin change interrupt" pin).
// Uno:       pins 2 and 3
// Mega 2560: 2, 3, 21, 20, 19, and 18
//
// The Button() argument is simply the pin the button is connected to, that is, wherever
// pin 1 of the Breakout board is connected (also connect the grounds).
// No worries about PWM or interrupt pins here.
//
//Motor m = Motor(3, 4, 10, 2, 5);
//Button b = Button(7);


void setup()
{
  // Only call this if you are using a Bricktronics Shield,
  // otherwise leave it commented-out.
  //Bricktronics::begin();

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

