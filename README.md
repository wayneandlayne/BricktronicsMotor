BricktronicsMotor
=================

**BricktronicsMotor v1.2 - A software library for LEGO NXT motors.**

More details at http://www.wayneandlayne.com/bricktronics/

**To Download**
* Click the "Clone or download" button on the upper-right side of this page below the pink bar, then select "Download ZIP". Rename the uncompressed folder BricktronicsMotor. Check that the BricktronicsMotor folder contains BricktronicsMotor.cpp and BricktronicsMotor.h
* Place the BricktronicsMotor library folder into your `<arduinosketchfolder>/libraries/` folder. You may need to create the libraries subfolder if this is your first installed library. Restart the Arduino IDE.

**API Highlights**
* `BricktronicsMotor(uint8_t enPin, uint8_t dirPin, uint8_t pwmPin, uint8_t encoderPin1, uint8_t encoderPin2)` - Constructor
* `void begin(void)` - Call the begin function in your setup() function
* `void coast(void)` - Let the motor coast
* `void brake(void)` - Rapidly slow down the motor
* `void hold(void)` - Stop the motor and hold it in place
* `void update(void)` - Recalculate the PID motor control parameters
* `void setFixedDrive(int16_t speed)` - Raw, uncontrolled motor speed setting
* `void goToPosition(int32_t position)` - Uses PID algorithm to drive motor to position
* `void goToPositionWaitForDelay(int32_t position, uint32_t delayMS)` - Same as goToPosition but wait for delayMS
* `void goToPositionWaitForArrival(int32_t position)` - Same as goToPosition but wait for arrival. May get stuck if motor never arrives...
* `bool goToPositionWaitForArrivalOrTimeout(int32_t position, uint32_t timeoutMS)` - Same as above but will timeout after timeoutMS.
* `void goToAngle(int32_t angle)` - There is a whole family of functions for moving to an angle (0 - 355 degrees)
* `void pidSetTunings(double Kp, double Ki, double Kd)` - Update the PID tuning parameters
* `bool settledAtPosition(int32_t position)` - Check if the motor has reached the desired position, accounting for PID output and a deadband around the desired position.
* More API details in [API.md](API.md)


**If you want to use motors with your Bricktronics Shield or Megashield, you may also be interested in these libraries:**
* [BricktronicsShield Arduino Library v1.2](https://github.com/wayneandlayne/BricktronicsShield)
* [BricktronicsMegashield Arduino Library v1.2](https://github.com/wayneandlayne/BricktronicsMegashield)

_Wayne and Layne, LLC and our products are not connected to or endorsed by the LEGO Group. LEGO, Mindstorms, and NXT are trademarks of the LEGO Group._

