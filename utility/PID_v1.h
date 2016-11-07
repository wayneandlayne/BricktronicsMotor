#ifndef PID_V1_H
#define PID_V1_H

#define LIBRARY_VERSION 1.1.1

// Constants used in some of the functions below
#define MANUAL      0
#define AUTOMATIC   1

#define DIRECT      0
#define REVERSE     1

typedef enum
{
    InputMode_Direct,
    InputMode_Derivative,
} input_mode_e;

class PID
{
    public:
    // Commonly used functions:

    // Constructor. Links the PID to the Input, Output, and Setpoint. Initial tuning parameters are also set here.
    PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, int ControllerDirection);

    // Sets PID to either Manual (0) or Auto (non-0)
    void SetMode(int Mode);

    // Sets input mode to either InputMode_Direct or InputMode_Derivative
    void SetInputMode(input_mode_e InputMode);

    // Perform the PID calculation. Should be called every time loop() cycles.
    // ON/OFF and calculation frequency can be set using SetMode and SetSampleTime, respectively.
    bool Compute(void); 

    // Clamp the output to a specific range. 0-255 by default, but it's likely the user will want to change this depending on the application.
    void SetOutputLimits(double OutputMin, double OutputMax); 


    // Available but not commonly used functions:

    void SetTunings(double Kp , double Ki,      // * While most users will set the tunings once in the
                    double Kd);                 //   constructor, this function gives the user the option
                                                //   of changing tunings during runtime for Adaptive control.
    void SetControllerDirection(int Direction); // * Sets the Direction, or "Action" of the controller. DIRECT
                                                //   means the output will increase when error is positive. REVERSE
                                                //   means the opposite. It's very unlikely that this will be needed
                                                //   once it is set in the constructor.
    void SetSampleTime(int NewSampleTime);      // * Sets the minimum time between two PID calculations.  Default is 100 milliseconds.

    // Display functions
    double GetKp(void);         // These functions query the pid for interal values.
    double GetKi(void);         // they were created mainly for the pid front-end,
    double GetKd(void);         // where it's important to know what is actually
    int GetMode(void);          // inside the PID.
    int GetDirection(void);

    private:
    void Initialize();

    double dispKp;              // We'll hold on to the tuning parameters in user-entered
    double dispKi;              // format for display purposes
    double dispKd;

    double kp;                  // (P)roportional Tuning Parameter
    double ki;                  // (I)ntegral Tuning Parameter
    double kd;                  // (D)erivative Tuning Parameter

    int controllerDirection;

    double *myInput;            // Pointers to the Input, Output, and Setpoint variables
    double *myOutput;           // This creates a hard link between the variables and the
    double *mySetpoint;         // PID, freeing the user from having to constantly tell us
                                // what these values are. With pointers we'll just know.

    unsigned long lastTime;
    double ITerm, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;

    /* To support speed control with PID, we added a configuration here that
     * changes Compute() to use dInput/dTime instead of just input. */
    input_mode_e inputMode;
};
#endif /* PID_V1_H */

