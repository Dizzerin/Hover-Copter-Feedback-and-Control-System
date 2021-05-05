#include <Six302.h>           // https://github.com/almonds0166/6302view
#include <Encoder.h>          // Ai Esp32 Rotary Encoder by Igor Antolic

// Pin Definitions
const int motorPin = 5;       // Motor output pin -- GPIO pin 5 -- pin 29
const int potPinA = 25;       // Motor output pin -- GPIO pin 25 -- pin 9
const int potPinB = 26;       // Motor output pin -- GPIO pin 26 -- pin 10
const int ADCPinHigh = 8;     // ADC input pin used for high voltage side of current measuring diode -- GPIO pin 33 -- pin 8
const int ADCPinLow = 7;      // ADC input pin used for low voltage side of current measuring diode -- GPIO pin 32 -- pin 7
const int encoderPinA = 17;   // Shaft angle encoder input pin A  -- GPIO pin 17 -- pin 17  (used to be GPIO pin 0 -- pin 33)
const int encoderPinB = 16;   // Shaft angle encoder input pin B  -- GPIO pin 16 -- pin 16  (used to be GPIO pin 2 -- pin 34)

// Initialize Vars
//uint16_t PWMDutyCycle = 0;
float PWMDutyCycle = 0.0;       // Must be a float for 6302view to be able to control it
const int PWMFreq = 10000;      // 10KHz
const int PWMChannel = 0;   
const int PWMResolution = 12;   // 12-bits
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) -1);
long encoderPos = -999;         // Encoder position
float reportedEncoderPos = 0.0; // Reported encoder position (6302view will only accept a float)
const float Resistor = 0.5;     // Resistor value in ohms (R3 on schematic, resistor used for measuing current to motor)
int ADCValueHigh = 0;           // ADC input value from high voltage side of R3
int ADCValueLow = 0;            // ADC input value from low voltage side of R3
float VoltageHigh = 0.0;        // High voltage side of R3
float VoltageLow = 0.0;         // Low voltage side of R3
float motorCurrent = 0.0;       // Current being delivered to the motor

// Initilization for encoder reading
Encoder myEncoder(encoderPinA, encoderPinB);

// Initialization for 6302view
#define STEP_TIME 1000    // Time between loops/steps in microseconds
#define REPORT_TIME 10000 // Time between each report in microseconds
CommManager comManager(STEP_TIME, REPORT_TIME);


void setup() {
   // Add modules (general format: pointer to var, title, other options)
   comManager.addSlider(&PWMDutyCycle, "PWM Duty Cycle", 0, MAX_DUTY_CYCLE, 1);
   comManager.addPlot(&reportedEncoderPos, "Encoder Output", -4095, 4095);             // Not sure about actual range yet
   comManager.addPlot(&motorCurrent, "Motor Current (Amps)", 0, 7);
   
   // Connect to 6302view via serial communication
   comManager.connect(&Serial, 115200);

  // Setup PWM channels, frequency, and resolution
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  // Attach PWM channel to GPIO pin
  ledcAttachPin(motorPin, PWMChannel);
}

float get_motor_current(){
  // Read values from ADC
  ADCValueHigh = analogRead(ADCPinHigh);
  ADCValueLow = analogRead(ADCPinLow);

  // Convert to proper voltage values
  // The ESP32 ADC has a resolution of 12-bits (0-4095) with 4095 corresponding to 3.3V
  VoltageHigh = (ADCValueHigh * 3.3) / 4096;
  VoltageLow = (ADCValueLow * 3.3) / 4096;

  // Calculate and return motor current
  return ((VoltageHigh - VoltageLow)/Resistor);
}

void loop() {

  // Update vars
  encoderPos = myEncoder.read();
  reportedEncoderPos = float(encoderPos);
  motorCurrent = get_motor_current();
  
  // Update 6302view
  comManager.step();

  // Update/Set PWM output with desired duty cycle according to GUI slider
  ledcWrite(PWMChannel, PWMDutyCycle);
}
