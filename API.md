# BricktronicsMotor API

This library interfaces with LEGO NXT and EV3 motors. It can be used with the [Bricktronics Shield](https://store.wayneandlayne.com/products/bricktronics-shield-kit.html), [Bricktronics Megashield](https://store.wayneandlayne.com/products/bricktronics-megashield-kit.html), or standalone with the [Bricktronics Motor Driver](https://store.wayneandlayne.com/products/bricktronics-motor-driver.html). For the shield/megashield, use the constructor below with the `BricktronicsMotorSettings` struct, otherwise use the constructor below that accepts the raw pin numbers.

# Connection with [Bricktronics Shield](https://store.wayneandlayne.com/products/bricktronics-shield-kit.html)

Use a motor with any motor port on the Bricktronics Shield.

Constructor usage for Bricktronics Shield:
```C++
#include <BricktronicsShield.h>
#include <BricktronicsMotor.h>
BricktronicsMotor m(BricktronicsShield::MOTOR_1);
```

# Connection with [Bricktronics Megashield](https://store.wayneandlayne.com/products/bricktronics-megashield-kit.html)

Use a motor with any motor port on the Bricktronics Megashield.

Constructor usage for Bricktronics Megashield:
```C++
#include <BricktronicsMegashield.h>
#include <BricktronicsMotor.h>
BricktronicsMotor m(BricktronicsMegashield::MOTOR_5);
```

# Connection with [Bricktronics Motor Driver](https://store.wayneandlayne.com/products/bricktronics-motor-driver.html)

Use a motor with either motor port on the Bricktronics Motor Driver. To power the circuit board and the motors, provide ground connections (GND) plus 5v (VCC) and your motor voltage (VM). LEGO NXT motors can be driven with up to 9v. Each motor port has five connections:

* Enable (EN) - Drive this pin low to disable the motor drivers, drive this pin high to enable motor drivers.
* Direction (DIR) - Setting this pin high/low will drive the motor either clockwise or counter-clockwise.
* Speed (PWM) - Once you enable the motor drivers and have set your direction, provide a pulse-width modulated signal to this pin to control the motor’s speed.
* Encoder Tachometer pins (T1 and T2) - These two pins provide a quadrature encoded signal that tracks the motor’s rotation.

Constructor usage for Bricktronics Breakout Board:
```C++
#include <BricktronicsMotor.h>
// Constructor arguments are: EN, DIR, PWM, T1, T2
BricktronicsMotor m(3, 4, 10, 2, 5);
```

# Quick Example

```C++
#include <BricktronicsMotor.h>

// Use one of the constructor options listed above.
BricktronicsMotor m(3, 4, 10, 2, 5);

void setup()
{
    Serial.begin(115200);
    // If using a Bricktronics Shield, you need to call
    // BricktronicsShield::begin();
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
```

# Constructors and `begin()`

#### `BricktronicsMotor(uint8_t enPin, uint8_t dirPin, uint8_t pwmPin, uint8_t encoderPin1, uint8_t encoderPin2)`

Constructor - Simple constructor that accepts the five motor pins

**Parameters**

* `uint8_t enPin` - The Arduino pin number where the EN signal is connected.
* `uint8_t dirPin` - The Arduino pin number where the DIR signal is connected.
* `uint8_t pwmPin` - The Arduino pin number where the PWM signal is connected.
* `uint8_t encoderPin1` - The Arduino pin number where the T1 signal is connected.
* `uint8_t encoderPin2` - The Arduino pin number where the T2 is connected.


#### `BricktronicsMotor(const BricktronicsMotorSettings &settings)`

Constructor - Advanced constructor that accepts a BricktronicsMotorSettings struct to also override the low-level Arduino functions.

**Parameters**

* `const BricktronicsMotorSettings &settings` - A const reference to the struct containing all the motor settings. Get these structs from the [BricktronicsShield](https://github.com/wayneandlayne/BricktronicsShield) or [BricktronicsMegashield](https://github.com/wayneandlayne/BricktronicsMegashield) library.

#### `void begin(void)`

Set up the motor library internals and pin modes. Sets the motor to coast. Call this function once for each motor instance during your setup() function.


# Basic API functions

#### `void coast(void)`

Disconnects the motor windings. Excess back-EMF will be shunted through the motor driver's protection diodes and/or the body diodes in the H-bridge. This will not actively slow-down the motor.


#### `void brake(void)`

Shorts the motor windings, which will quickly bring it to a stop. This mode does not lock the motor in place electrically or mechanically. You may also be interested in the hold() function below.


#### `void hold(void)`

Similar to brake(), but this function sets up a goToPosition() for the current position, effectively locking the motor in place. That is, it will resist any efforts to turn the motor, and will constantly try to restore the motor to the position it had when you called hold(). Just like with goToPosition(), you need to periodically call update().


#### `int32_t getPosition(void)`

Read the encoder's current position, as a signed 32-bit number.

#### `void setPosition(int32_t pos)`

Write the encoder's current position - This will mess up any PID control in progress! This only sets the number corresponding to the motor's current position. Usually you just want to reset the position to zero.


#### `void update(void)`

Some of the functions below need to periodically check on the motor's operation and update how fast and/or which direction to drive the motor. Use this update() function to do that. Call this function as often as you can, since it will only actually update as often as the frequency setpoint (defaults to 50ms), which can be updated below.


#### `void delayUpdateMS(uint32_t delayMS)`

This function periodically calls update() until delayMS milliseconds have elapsed. Useful if you have nothing else to do but sit and wait for delayMS while updating the motor's PID algorithm.


# Raw, uncontrolled speed settings

#### `void setFixedDrive(int16_t s)`

Sets the raw motor drive strength. There is no monitoring or control of the speed here, just set a fixed drive strength between -255 and +255.

#### `int16_t getFixedDrive(void)`

Retrieves the previously-set fixed drive speed.


# Position control functions

#### `void goToPosition(int32_t position)`

Switches PID control into position-tracking mode, and sets the desired motor position to the first argument. You need to periodically call update() in order for PID modes to work correctly.

#### `void goToPositionWaitForDelay(int32_t position, uint32_t delayMS)`

Go to the specified position using PID, but wait for the specified number of milliseconds before returning.

#### `void goToPositionWaitForArrival(int32_t position)`

Go to the specified position using PID, but wait until the motor arrives. Can be vulnerable to getting stuck forever if the motor never reaches the desired position.

#### `bool goToPositionWaitForArrivalOrTimeout(int32_t position, uint32_t timeoutMS)`

Same as goToPositionWaitForArrival above, but return after timeoutMS milliseconds in case it gets stuck. Returns true if we made it to position, false if we had a timeout.


# Angle control functions

These are the angle control functions (0 - 359 degrees), that handle discontinuity nicely. Can specify any angle, positive or negative. If you say "go to angle 721" it will be the same as "go to angle 1". Similarly, "go to angle -60" will be "go to angle 300". If you want "go 45 degrees clockwise from here", try using m.goToAngle(m.getAngle() + 45);

#### `void goToAngle(int32_t angle)`

Sets desired motor angle and sets the motor for PID mode. You need to periodically call update() in order for PID modes to work correctly.

#### `void goToAngleWaitForDelay(int32_t angle, uint32_t delayMS)`

Go to the specified angle using PID, but wait for the specified number of milliseconds before returning.

#### `void goToAngleWaitForArrival(int32_t angle)`

Go to the specified angle using PID, but wait until the motor arrives. Can be vulnerable to getting stuck forever if the motor never reaches the desired angle.

#### `bool goToAngleWaitForArrivalOrTimeout(int32_t angle, uint32_t timeoutMS)`

Same as goToAngleWaitForArrival above, but return after timeoutMS milliseconds in case it gets stuck. Returns true if we made it to angle, false if we had a timeout.

#### `uint16_t getAngle(void)`

Returns the current angle, in the range of (0 - 359) degrees.

#### `void setAngle(int32_t angle)`

Updates the current encoder position to be the specified angle.

#### `void setAngleOutputMultiplier(int8_t multiplier)`

For the angle control, the user can specify a different multiplier between motor encoder ticks and "output rotations", defaults to 1. Use this setting if your motor is connected to a gear train that makes a different number of motor rotations per output rotation.

For example, if you have a 5:1 gear train between your motor and the final output, then you can specify this value as 5. Negative numbers should work just fine.


# PID functions

#### `void pidSetUpdateFrequencyMS(int timeMS)`

Update the maximum frequency at which the PID algorithm will actually update. Defaults to 50.

#### `void pidPrintValues(void)`

Print out the PID values to the serial port, including the setpoint, the input, and the output.

#### `double pidGetKp(void)`

Return the PID proportional tuning parameter Kp.

#### `double pidGetKi(void)`

Return the PID integral tuning parameter Ki.

#### `double pidGetKd(void)`

Return the PID derivative tuning parameter Kd.

#### `void pidSetTunings(double Kp, double Ki, double Kd)`

Set the three PID tuning parameters.

#### `void pidSetKp(double Kp)`

Set the PID proportional tuning parameter Kp.

#### `void pidSetKi(double Ki)`

Set the PID proportional tuning parameter Ki.

#### `void pidSetKd(double Kd)`

Set the PID proportional tuning parameter Kd.

#### `bool settledAtPosition(int32_t position)`

Motors have some slop in their encoder output readings, so this function can be used to make a "close enough?" check. The epsilon value can be get/set using the functions below, and is used in the settledAtPosition check. This function also checks to ensure that the PID algorithm has settled down enough (that is, _pidOutput < BRICKTRONICS_MOTOR_PID_OUTPUT_SETTLED_THRESHOLD) that we can just brake() without having to worry about coasting through the setpoint.


#### `void setEpsilon(uint8_t epsilon)`

Sets the epsilon value used in settledAtPosition() above.

#### `uint8_t getEpsilon(void)`

Gets the epsilon value used in settledAtPosition() above.
