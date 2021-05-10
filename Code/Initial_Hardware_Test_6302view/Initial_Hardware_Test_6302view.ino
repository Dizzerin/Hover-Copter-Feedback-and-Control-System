/* Info
 * Author: Caleb Nelson
 * Revision: 0.4
 * Last Edit: 5/9/2021
 * 
 * Python Dependancies (dependencies of 6302view software)
 *  pyserial
 *  websockets
 *  
 * Description
 *  This program is used to control a hover copter arm for feedback and control systems offered at Walla Walla University
 *  It currently doesn't implement any control algorithms as it is currently in the development stage still
 */
#include <Six302.h>           // https://github.com/almonds0166/6302view
#include <Encoder.h>          // Encoder Library
#include <RunningAverage.h>   // Used for keeping a running average of the voltage readings to keep out sensor noise

// Trig Definitions
// It appears the first three are now built in
//#define PI 3.1415926535897932384626433832795
//#define HALF_PI 1.5707963267948966192313216916398
//#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886    // PI/180
#define RAD_TO_DEG 57.295779513082320876798154814105      // 180/PI

// Pin Definitions
const int motorPin = 5;       // Motor output pin -- GPIO pin 5 -- pin 29
const int potPinA = 25;       // Motor output pin -- GPIO pin 25 -- pin 9
const int potPinB = 26;       // Motor output pin -- GPIO pin 26 -- pin 10
const int ADCPinHigh = 33;    // ADC input pin used for high voltage side of current measuring diode -- GPIO pin 33 -- pin 8
const int ADCPinLow = 32;     // ADC input pin used for low voltage side of current measuring diode -- GPIO pin 32 -- pin 7
const int encoderPinA = 17;   // Shaft angle encoder input pin A -- GPIO pin 17 -- pin 17 (used to be GPIO pin 0 -- pin 33)
const int encoderPinB = 16;   // Shaft angle encoder input pin B -- GPIO pin 16 -- pin 16 (used to be GPIO pin 2 -- pin 34)

// Basic Parameters
const float resistor = 0.5;     // Resistor value in ohms (R3 on schematic, resistor used for measuing current to motor)
//const float armLength = 0;
//const float armMass = 0;
//const float motorMass = 0;
//const float inertia = 0;
//const float viscousFriction = 0;  // b
//const float drag = 0;

// State Feeback Controller Parameters
const float k1 = 3.8093;
const float k2 = 0.5944;

// 6302view Initialization
#define STEP_TIME 5000              // Time between loops/steps in microseconds
#define REPORT_TIME 50000           // Time between each report in microseconds
CommManager comManager(STEP_TIME, REPORT_TIME);

// PWM Initialization
float PWMDutyCycleLarge = 0.0;  // Large signal portion of PWM duty cycle (must be a float for 6302view to be able to control it)
float PWMDutyCycleSmall = 0.0;  // Small signal portion of PWM duty cycle
float PWMDutyCycle = 0.0;       // Total PWM duty cycle
const int PWMFreq = 78000;      // 78KHz -- maximum frequency is 80000000 / 2^bit_num
const int PWMChannel = 0;       // PWM channel
const int PWMResolution = 10;   // 10-bits
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution)-1);

// Encoder and Angle Related Initialization
Encoder myEncoder(encoderPinA, encoderPinB);  // Initilization for encoder reading
int currentEncoderPos = 0;                    // Current encoder position
float lastTheta = 0.0;                        // Last hover arm angle in radians
float currentTheta = 0.0;                     // Current hover arm angle in radians
float currentThetaDegree = 0.0;               // Current hover arm angle in degrees
float thetaDot = 0.0;                         // Approximate derivate of angle in radians per second

// Electical ADC Voltage and Current Related Initialization
int ADCValueHigh = 0;           // ADC input value from high voltage side of R3
int ADCValueLow = 0;            // ADC input value from low voltage side of R3
int numAvgSamples = 10;         // Number of values to use for the running averages
RunningAverage voltageHigh(numAvgSamples); // High voltage side of R3
RunningAverage voltageLow(numAvgSamples);  // Low voltage side of R3
float voltageHighFloat = 0.0;   // High voltage side of R3
float voltageLowFloat = 0.0;    // Low voltage side of R3
float voltageDrop = 0.0;        // Voltage drop across R3
float motorCurrent = 0.0;       // Current being delivered to the motor

// Other Initialization
int deltaT = STEP_TIME;         // Time span between samples in miliseconds -- used for calculating estimated time derivatives -- currently estimated as explicit sleep time, but could implement timer later
bool controlOn = false;         // Toggle for enabling feedback control algorithms
bool setZeroPoint = false;      // Button to set the current hover arm position as the 0 point (theta=0 should correspond to horizontal)


void setup() {
   // Add modules for 6302view (general format: pointer to var, title, other options)
   // Buttons
   comManager.addButton(&setZeroPoint, "Set 0 point");
   // Toggles
   comManager.addToggle(&controlOn, "Enable Feedback Control");
   // Sliders
   comManager.addSlider(&PWMDutyCycleLarge, "PWM Duty Cycle", 0, MAX_DUTY_CYCLE, 1);   // Slider to control PWM/Duty Cycle
   // Numbers
   comManager.addNumber(&voltageDrop, "R3 Voltage Drop (V)");                     // Max of about 1 volt
   comManager.addNumber(&motorCurrent, "Motor Current (Amps)");                   // Max of about 2 amps
   comManager.addNumber(&currentThetaDegree, "Theta (deg)");                      // Current hover arm angle in degrees
   comManager.addNumber(&thetaDot, "Theta Dot (rad/s)");                          // Derivative of hover arm position in radians per second   
   // Plots
   comManager.addPlot(&currentThetaDegree, "Theta (deg)", -360, 360);             // Current hover arm angle in degrees
   comManager.addPlot(&thetaDot, "Theta Dot (rad/s)", -4*PI, 4*PI);               // Derivative of hover arm position in radians per second
//   comManager.addPlot(&motorCurrent, "Motor Current (Amps)", 0, 2.5);             // Max of about 2 amps
//   comManager.addPlot(&voltageDrop, "R3 Voltage Drop (V)", 0, 1.1);               // Max of about 1 volt
//   comManager.addPlot(&voltageHighFloat, "High side Voltage (V)", 0, 3.5);        // Max of about 3.3 volts
//   comManager.addPlot(&voltageLowFloat, "Low side Voltage (V)", 0, 2.5);          // Max of about 2.3 volts
//   comManager.addPlot(&deltaT, "Delta T (ms)",  0, 8000);                         // Time span between samples (should be constant STEP_TIME) -- At some point I want to try implementing a timer and timing each loop and seeing how much it varies
    
   // Connect to 6302view via serial communication
   comManager.connect(&Serial, 115200);

  // Setup PWM channels, frequency, and resolution
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  // Attach PWM channel to GPIO pin
  ledcAttachPin(motorPin, PWMChannel);

  // Explicitly clear running averages
  voltageHigh.clear();
  voltageLow.clear();
}


float get_motor_current(){
  // Read values from ADC
  ADCValueHigh = analogRead(ADCPinHigh);
  ADCValueLow = analogRead(ADCPinLow);

  // Convert to proper voltage values
  // The ESP32 ADC has a resolution of 12-bits (0-4095) with 4095 corresponding to 3.3V
  voltageHigh.addValue((ADCValueHigh * 3.3) / 4096);
  voltageLow.addValue((ADCValueLow * 3.3) / 4096);

  // For 6302view - get the floats and work with those, also avoid repeated function calls
  voltageHighFloat = voltageHigh.getAverage();
  voltageLowFloat = voltageLow.getAverage();
  voltageDrop = voltageHighFloat - voltageLowFloat;


  // Calculate and return motor current
  return ((voltageHighFloat - voltageLowFloat)/resistor);
}


void loop() {
  // Update vars
  lastTheta = currentTheta;
  currentEncoderPos = -1 * myEncoder.read();            // Note I made this negative becuase of the way our hover copter arm is setup.  Counter clockwise on the encoder is actually clockwise the way we look at it
  currentTheta = float(currentEncoderPos)*(2*PI)/10000; // Encoder value of 98900 corresponds to 2 pi radians or 360 degrees
  currentThetaDegree = currentTheta * RAD_TO_DEG;       // Convert to degrees for display
  motorCurrent = get_motor_current();
  // Recalculate derivative
  thetaDot = (lastTheta-currentTheta)/deltaT;
  
  // Implement control
  if (controlOn) {
    // Calculate new small signal portion used to update total duty cycle
    PWMDutyCycleSmall = k1 * currentTheta + k2 * thetaDot;
  }
  else {
    PWMDutyCycleSmall = 0;
  }

  // Add large and small signal components to get total updated duty cycle
  PWMDutyCycle = PWMDutyCycleLarge + PWMDutyCycleSmall;
    
  // Set updated PWM output with desired duty cycle
  ledcWrite(PWMChannel, PWMDutyCycle);

  // Check if zero point should be reset
  if (setZeroPoint) {
    myEncoder.write(0);   // Reset the current position as 0
  }

  // Update 6302view
  comManager.step();      // Includes explicit delay for STEP_TIME miliseconds
}
