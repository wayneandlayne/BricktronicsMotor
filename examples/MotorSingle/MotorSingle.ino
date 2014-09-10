// Bricktronics Example: MotorSingle
// http://www.wayneandlayne.com/bricktronics
// This example uses a LEGO NXT Motor.
//
// This example starts the motor at an intermediate speed,
// then speeds it up to full speed, and does the same in reverse.
//
// This example uses a motor, so it needs more power than a USB port can give.
// We really don't recommend running motors off of USB ports (they will be
// slow and sluggish, other things won't quite work right, things can get hot)
// it's just not a good idea.  Use an external power supply that provides
// between 7.2 and 9 volts DC, and can provide at least 600 mA per motor
// (1 amp preferably). Two options that work really well are a 9V wall adapter
// or a 6xAA battery pack (2.1mm plug, center positive).


// Include the Bricktronics Motor library
#include <Motor.h>


// This example can be run in three different ways. Pick one, and un-comment
// the code lines corresponding to your chosen method. Comment-out the lines
// for the other methods that you aren't using.

// 1. With a Bricktronics Shield - Include these lines and be sure to
// call Bricktronics::begin() in the setup() function below. Select the
// motor port (BS_MOTOR_1 or BS_MOTOR_2) in the constructor below.
//
//#include <Wire.h>
//#include <Bricktronics2.h>
//Motor m = Motor(Bricktronics::BS_MOTOR_1);

// 2. With a Bricktronics Megashield - Include these lines below but do not
// call Bricktronics::being() in the setup() function below. Select the
// desired motor port (BMS_MOTOR_1 through BMS_MOTOR_6) in the constructor below.
//
//#include <Bricktronics2.h>
//Motor m = Motor(Bricktronics::BMS_MOTOR_1);

// 3. With a Bricktronics Motor Driver - No additional #includes needed,
// just update the five pin assignments in the constructor below.
// The arguments are: enPin, dirPin, pwmPin, tachPinA, tachPinB
// There are a few considerations for pin assignments:
// A. pwmPin needs to be a pin with PWM capabilities (analogWrite)
// Uno:       pins 3, 5, 6, 9, 10, and 11
// Mega 2560: pins 2 to 13 and 44 to 46
// B. At least one of tachPinA/B needs to be an actual interrupt pin (not just
// a "pin change interrupt" pin).
// Uno:       pins 2 and 3
// Mega 2560: 2, 3, 21, 20, 19, and 18
//
//Motor m = Motor(3, 4, 10, 2, 5);


void setup()
{
  // Be sure to set your serial console to 115200 baud
  Serial.begin(115200);

  // Only call this line if you are using a Bricktronics Shield,
  // otherwise leave it commented-out.
  //Bricktronics::begin();

  // Initialize the motor connections
  m.begin();
}

void loop() 
{
  Serial.println("Going forward.");
  m.rawSetSpeed(75);
  delay(1000);
  
  m.rawSetSpeed(255);
  delay(1000);

  Serial.println("Going in reverse.");
  m.rawSetSpeed(-75);
  delay(1000);
  
  m.rawSetSpeed(-255);
  delay(1000);
}

