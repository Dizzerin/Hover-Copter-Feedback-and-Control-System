/* Info
 * Author: Caleb Nelson
 * Revision: 0.3
 * Last Edit: 5/6/2021
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

// Pin Definitions
const int motorPin = 5;       // Motor output pin -- GPIO pin 5 -- pin 29
const int potPinA = 25;       // Motor output pin -- GPIO pin 25 -- pin 9
const int potPinB = 26;       // Motor output pin -- GPIO pin 26 -- pin 10
const int ADCPinHigh = 33;     // ADC input pin used for high voltage side of current measuring diode -- GPIO pin 33 -- pin 8
const int ADCPinLow = 32;      // ADC input pin used for low voltage side of current measuring diode -- GPIO pin 32 -- pin 7
const int encoderPinA = 17;   // Shaft angle encoder input pin A -- GPIO pin 17 -- pin 17 (used to be GPIO pin 0 -- pin 33)
const int encoderPinB = 16;   // Shaft angle encoder input pin B -- GPIO pin 16 -- pin 16 (used to be GPIO pin 2 -- pin 34)

// Initialize Vars
//uint16_t PWMDutyCycle = 0;
float PWMDutyCycle = 0.0;       // Must be a float for 6302view to be able to control it
const int PWMFreq = 78000;      // 78KHz -- maximal frequency is 80000000 / 2^bit_num
const int PWMChannel = 0;       // PWM channel
const int PWMResolution = 10;   // 10-bits
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution)-1);
long encoderPos = 0;            // Initial encoder position
float reportedEncoderPos = 0.0; // Reported encoder position (6302view will only accept a float)
const float resistor = 0.5;     // Resistor value in ohms (R3 on schematic, resistor used for measuing current to motor)
int ADCValueHigh = 0;           // ADC input value from high voltage side of R3
int ADCValueLow = 0;            // ADC input value from low voltage side of R3
int numAvgSamples = 10;         // Number of values to use for the running averages
RunningAverage voltageHigh(numAvgSamples); // High voltage side of R3
RunningAverage voltageLow(numAvgSamples);  // Low voltage side of R3
float voltageHighFloat = 0.0;   // High voltage side of R3
float voltageLowFloat = 0.0;    // Low voltage side of R3
float voltageDrop = 0.0;        // Voltage drop across R3
float motorCurrent = 0.0;       // Current being delivered to the motor

// Initilization for encoder reading
Encoder myEncoder(encoderPinA, encoderPinB);

// Initialization for 6302view
#define STEP_TIME 5000    // Time between loops/steps in microseconds
#define REPORT_TIME 50000 // Time between each report in microseconds
CommManager comManager(STEP_TIME, REPORT_TIME);


void setup() {
   // Add modules (general format: pointer to var, title, other options)
   comManager.addSlider(&PWMDutyCycle, "PWM Duty Cycle", 0, MAX_DUTY_CYCLE, 1);
   comManager.addPlot(&reportedEncoderPos, "Encoder Output", -4095, 4095);             // Not sure about actual range yet
   comManager.addPlot(&motorCurrent, "Motor Current (Amps)", 0, 2.5);
   comManager.addPlot(&voltageDrop, "Motor Voltage (V)", 0, 5);

   // Other modules that may not stay
   comManager.addPlot(&voltageHighFloat, "High side Voltage (V)", 0, 3.5);
   comManager.addPlot(&voltageLowFloat, "Low side Voltage (V)", 0, 3.5);
   comManager.addNumber(&voltageDrop, "Motor Voltage (V)");
   comManager.addNumber(&motorCurrent, "Motor Current (Amps)");
   
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
  encoderPos = -1 * myEncoder.read();       // Note I made this negative becuase of the way our hover copter arm is setup.  Counter clockwise is positive the way we gnerally think of it
  reportedEncoderPos = float(encoderPos);
  motorCurrent = get_motor_current();
  
  // Update 6302view
  comManager.step();

  // Update/Set PWM output with desired duty cycle according to GUI slider
  ledcWrite(PWMChannel, PWMDutyCycle);
}
