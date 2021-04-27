#include <Six302.h>
#include <Encoder.h>

// Initialize Vars
uint16_t PWMDutyCycle;
const int motorPin = 5;     // Motor output pin -- GPIO pin 5 -- pin 29
const int potPinA = 25;     // Motor output pin -- GPIO pin 25 -- pin 9
const int potPinB = 26;     // Motor output pin -- GPIO pin 26 -- pin 10
const int PWMFreq = 10000;  // 10KHz
const int PWMChannel = 0;   
const int PWMResolution = 12; // 12-bits
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) -1);

// Initilization for encoder reading
const int encoderPinA = 0;  // Shaft angle encoder input pin A  -- GPIO pin 0 -- pin 33
const int encoderPinB = 2;  // Shaft angle encoder input pin B  -- GPIO pin 2 -- pin 34
Encoder myEncoder(encoderPinA, encoderPinB);
long oldEncoderPos = -999;
long newEncoderPos = 0;

// Initialization for 6302view
#define STEP_TIME 5000    // microseconds
#define REPORT_TIME 50000 // microseconds
CommManager cm(STEP_TIME, REPORT_TIME);
float input;
float output;

void setup() {
   /* Add modules */
   cm.addSlider(&input, "Input", 0, MAX_DUTY_CYCLE, 1);
   cm.addPlot(&output, "Output", -1, 26);

   /* Ready to communicate over serial */
   cm.connect(&Serial, 115200);


  // Setup PWM channels, frequency, and resolution
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  // Attach PWM channel to GPIO pin
  ledcAttachPin(motorPin, PWMChannel);
  
}


void loop() {

  // Update encoder value
  long newEncoderPos = myEncoder.read();
  if (newEncoderPos != oldEncoderPos) {
    oldEncoderPos = newEncoderPos;
    output = newEncoderPos;
  }
  cm.step();


  PWMDutyCycle = input;

  // Set PWM output with desired duty cycle according to GUI slider
  ledcWrite(PWMChannel, PWMDutyCycle);
}
