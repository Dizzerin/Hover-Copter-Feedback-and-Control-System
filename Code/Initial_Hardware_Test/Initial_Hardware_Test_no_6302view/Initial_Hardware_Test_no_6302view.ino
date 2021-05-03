#include <Encoder.h>

// Initialize Vars
uint16_t PWMDutyCycle = 0;
int delay_time = 3000;      // delay time in miliseconds between stepping PWM values
const int motorPin = 5;     // Motor output pin -- GPIO pin 5 -- pin 29
const int potPinA = 25;     // Motor output pin -- GPIO pin 25 -- pin 9
const int potPinB = 26;     // Motor output pin -- GPIO pin 26 -- pin 10
const int PWMFreq = 10000;  // 10KHz
const int PWMChannel = 0;   
const int PWMResolution = 12; // 12-bits
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) -1);

// Initilization for encoder reading
%const int encoderPinA = 0;  // Shaft angle encoder input pin A  -- GPIO pin 0 -- pin 33
%const int encoderPinB = 2;  // Shaft angle encoder input pin B  -- GPIO pin 2 -- pin 34
const int encoderPinA = 17;  // Shaft angle encoder input pin A  -- GPIO pin 17 -- pin 17
const int encoderPinB = 16;  // Shaft angle encoder input pin B  -- GPIO pin 16 -- pin 16
Encoder myEncoder(encoderPinA, encoderPinB);
long newEncoderPos = 0;


void setup() {
  // Setup PWM channels, frequency, and resolution
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  // Attach PWM channel to GPIO pin
  ledcAttachPin(motorPin, PWMChannel);
}

void loop() {

  // Update encoder value
  long newEncoderPos = myEncoder.read();

  // Reset PWM duty cycle if its at the max
  if (PWMDutyCycle >= MAX_DUTY_CYCLE){
    PWMDutyCycle = 0;
  }

  // Step PWM duty cycle by 10%
  PWMDutyCycle += 0.1*MAX_DUTY_CYCLE;
  
  // Set PWM output with desired duty cycle according to GUI slider
  ledcWrite(PWMChannel, PWMDutyCycle);

  // Delay
  delay(delay_time);
}
