/* Info
 * Author: Caleb Nelson
 * Revision: 0.9
 * Last Edit: 6/2/2021
 * 
 * Description
 *  This program is used to control a hover copter arm for feedback and control systems course offered at Walla Walla University
 *  It currently implements a basic state feedback control algorithm with "hand placed" pole locations, where we arbitrarily chose
 *  pole locations in the left half of the complex plane and used Matlab/Octave to find the necessary controller gain parameters to 
 *  force the system to have the poles we chose.  It also includes some pole locations which were placed using LQR.
 *  This version now also includes a full order observer to estimate our full state based only off measuring the angle theta.
 *  There are currently two different state approximations that can be used (choice is dependant on the "Observer" On/Off toggle on the dashboard)
 *  If the observer toggle is set to on, the observer's estimated state will be used, if it is set to off, the "manually" estimated state estimation
 *  will be used.  The "manual" state estimation uses the measured theta, and an approximation for thetaDot as (currentTheta-lastTheta)/deltaT
 *  
 * Note: If you get the compile error "ledcSetup was not declared in this scope" that indicates the ESP32 board is not selected.
 */
#include <Six302.h>           // https://github.com/almonds0166/6302view -- requires python packages pyserial and websockets
#include <Encoder.h>          // Encoder by Paul Stoffregen -- https://www.pjrc.com/teensy/td_libs_Encoder.html -- https://github.com/PaulStoffregen/Encoder
#include <RunningAverage.h>   // RunningAverage by Rob Tillaart -- https://github.com/RobTillaart/RunningAverage -- used for reducing ADC sensor noise

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
const float resistor = 0.5;   // Resistor value in ohms (R3 on schematic, resistor used for measuing current to motor)

// Mathematical state space model related matricies
float A[2][2] = {{0, 1}, {0, -0.41}};     // Continuous time A matrix from state space model
float B[2] = {0, 117.76623};              // Continuous time B matrix from state space model
// Sates and measurements -- the state matricies are defined as [theta, thetaDot] in radians and radians/sec respsectively.
float measuredTheta = 0.0;                // Measured angle theta in radians
float lastEstimatedX[2] = {0.0, 0.0};     // Last estimated full state x -- this estimate is based on manual calculations approximating thetaDot as (currentTheta-lastTheta)/deltaT
float currentEstimatedX[2] = {0.0, 0.0};  // Current estimated full state x -- this estimate is based on manual calculations approximating thetaDot as (currentTheta-lastTheta)/deltaT
float lastXHat[2] = {0.0, 0.0};           // Last observer estimated state x -- xHat is the estimated state matrix from the observer
float currentXHat[2] = {0.0, 0.0};        // Current observer estimated state x -- xHat is the estimated state matrix from the observer
float usedX[2] = {0.0, 0.0};              // State that will be used for calculations, this allows us to switch between the two methods of full state estimation above
float estimatedXDiff[2] = {0.0, 0.0};     // Difference between measured state x and observer estimated state xHat.  Claculted as currentEstimatedX-currentXHat

// Control Gain Matrix Parameters for State Feeback Controller -- used to place the poles/eigenvalues at desired location -- obtained using Matlab or Octave
// float G[2] = {3.8093, 0.5944};   // Konrads manual placed poles
// float G[2] = {0.39259, 0.065299};   // Our 1st manual placed poles
// float G[2] = {3.12, 0.065};      // Our 2nd manual placed poles
float G[2] = {4.0, 0.95};        // Our 3rd manual placed poles
// float G[2] = {0.3162, 0.1205};   // Our first LQR placed poles

// Gain Matrix Parameters for the observer -- to place the poles/eigenvalues of the observer at desired location -- makes error go to 0 and xhat converge to x
float K[2] = {20.59, 101.558};      // Manually placed poles - poles at -10, -11

// 6302view Initialization
#define STEP_TIME 5000              // Time between loops/steps in microseconds
#define REPORT_TIME 50000           // Time between each report in microseconds
CommManager comManager(STEP_TIME, REPORT_TIME);

// PWM Initialization
float PWMDutyCycleLarge = 0.0;      // Large signal portion of PWM duty cycle (must be a float for 6302view to be able to control it)
float PWMDutyCycleSmall = 0.0;      // Small signal portion of PWM duty cycle
float PWMDutyCycleTotal = 0.0;      // Total PWM duty cycle
const int PWMFreq = 78000;          // 78KHz -- maximum frequency is 80000000 / 2^bit_num
const int PWMChannel = 0;           // PWM channel
const int PWMResolution = 10;       // 10-bits
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution)-1);

// Encoder and Angle Related Initialization
Encoder myEncoder(encoderPinA, encoderPinB);  // Initilization for encoder reading
int currentEncoderPos = 0;                    // Current encoder position
float currentThetaDegree = 0.0;               // Current hover arm angle in degrees

// Electical ADC Voltage and Current Related Initialization
int ADCValueHigh = 0;                         // ADC input value from high voltage side of R3
int ADCValueLow = 0;                          // ADC input value from low voltage side of R3
int numAvgSamples = 10;                       // Number of values to use for the running averages
RunningAverage voltageHigh(numAvgSamples);    // High voltage side of R3
RunningAverage voltageLow(numAvgSamples);     // Low voltage side of R3
float voltageHighFloat = 0.0;                 // High voltage side of R3
float voltageLowFloat = 0.0;                  // Low voltage side of R3
float voltageDrop = 0.0;                      // Voltage drop across R3
float motorCurrent = 0.0;                     // Current being delivered to the motor

// Other Initialization
//float deltaT = STEP_TIME/(10^6);    // Time span between samples in seconds -- used for calculating estimated time derivatives -- 6302's communication manager handles this and ensures this delay between loops/steps
float deltaT = 0.005;                 // Time span between samples in seconds -- used for calculating estimated time derivatives -- 6302's communication manager handles this and ensures this delay between loops/steps
bool controlOn = false;               // Toggle for enabling feedback control algorithms
bool powerOn = false;                 // Toggle to control power to the motor
bool observerOn = false;              // Toggle to control observer being used
bool setZeroPoint = false;            // Button to set the current hover arm position as the 0 point (theta=0 should correspond to horizontal)


void setup() {
  // Add modules for 6302view (general format: pointer to var, title, other options)
  // Buttons
  comManager.addButton(&setZeroPoint, "Set 0 point");
  // Toggles
  comManager.addToggle(&controlOn, "Feedback Control");
  comManager.addToggle(&powerOn, "Power");
  comManager.addToggle(&observerOn, "Observer");
  // Numbers
// comManager.addNumber(&voltageDrop, "R3 Voltage Drop (V)");                     // Max of about 1 volt
// comManager.addNumber(&motorCurrent, "Motor Current (Amps)");                   // Max of about 2 amps
  comManager.addNumber(&currentThetaDegree, "Theta Angle (deg)");                 // Current hover arm angle in degrees
// comManager.addNumber(&currentEstimatedX[1], "Theta Dot (rad/s)");               // Derivative of hover arm position in radians per second
// comManager.addNumber(&deltaT, "Delta T (seconds)");                            // Time span between samples (should be constant based on STEP_TIME)
  // Plots
  comManager.addPlot(&currentThetaDegree, "Theta Angle (deg)", -100, 100);          // Current hover arm angle in degrees
  comManager.addPlot(&estimatedXDiff[0], "Measured Theta - Observer Theta (deg)", -PI, PI);  // Difference between the measured and the observer estimated theta
  comManager.addPlot(&estimatedXDiff[1], "Estimated ThetaDot - Observer ThetaDot (rad/s)", -10, 10);  // Difference between the manually estimated and the observer estimated thetaDot
  comManager.addPlot(&currentEstimatedX[1], "Estimated Theta Dot (rad/s)", -8, 8);                  // Derivative of hover arm position in radians per second
  comManager.addPlot(&currentXHat[1], "Observer Theta Dot (rad/s)", -8, 8);                  // Derivative of hover arm position in radians per second
//  comManager.addPlot(&motorCurrent, "Motor Current (Amps)", 0, 2.5);             // Max of about 2 amps
//  comManager.addPlot(&voltageDrop, "R3 Voltage Drop (V)", 0, 1.1);               // Max of about 1 volt
//  comManager.addPlot(&voltageHighFloat, "High side Voltage (V)", 0, 3.5);        // Max of about 3.3 volts
//  comManager.addPlot(&voltageLowFloat, "Low side Voltage (V)", 0, 2.5);          // Max of about 2.3 volts
  comManager.addPlot(&PWMDutyCycleSmall, "Small Signal Duty Cycle", -3000, 3000);
  comManager.addPlot(&PWMDutyCycleTotal, "Total Duty Cycle", -100, 3000);
  // Sliders
  comManager.addSlider(&PWMDutyCycleLarge, "PWM Duty Cycle", 0, MAX_DUTY_CYCLE, 1);   // Slider to control PWM/Duty Cycle
  comManager.addSlider(&G[0], "G1", 0, 4, 0.05);
  comManager.addSlider(&G[1], "G2", 0, 3, 0.05);
  comManager.addSlider(&K[0], "K1", 0, 105, 1);
  comManager.addSlider(&K[1], "K2", 0, 105, 1);
    
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
  // Preserve old data
  lastEstimatedX[0] = currentEstimatedX[0];
  lastEstimatedX[1] = currentEstimatedX[1];
  lastXHat[0] = currentXHat[0];
  lastXHat[1] = currentXHat[1];

  // Get new measurements
  currentEncoderPos = -1 * myEncoder.read();              // Note: This is negative becuase of the way our hover copter arm is setup.  Counter clockwise on the encoder is actually clockwise the way we look at it
  measuredTheta = float(currentEncoderPos)*(2*PI)/10000;  // Encoder value of 10000 corresponds to 2 pi radians or 360 degrees
  currentThetaDegree = measuredTheta * RAD_TO_DEG;        // Convert to degrees for display
  // motorCurrent = get_motor_current();

  // Update manually estimated state -- this uses the measured theta and a derivative approximation for thetaDot
  currentEstimatedX[0] = measuredTheta;                                    // measured theta in radians
  currentEstimatedX[1] = (currentEstimatedX[0]-lastEstimatedX[0])/deltaT;  // approximate thetaDot in radians/sec
  
  // Update observer estimated state
  //   Also set which state estimation method will be used (manual estimation or observer estimation)
  if (observerOn){    
    // Update observer values
    currentXHat[0] = ((A[0][0]-K[0]-B[0]*G[0])*deltaT+1)*lastXHat[0]+((A[0][1]-B[0]*G[1])*deltaT)*lastXHat[1]+K[0]*deltaT*measuredTheta;
    currentXHat[1] = ((A[1][0]-K[1]-B[1]*G[0])*deltaT)*lastXHat[0]+((A[1][1]-B[1]*G[1])*deltaT+1)*lastXHat[1]+K[1]*deltaT*measuredTheta;
    
    // Set the obsever estimated state as the one to be used
    usedX[0] = currentXHat[0];
    usedX[1] = currentXHat[1];
  }
  else {
    // Set the manually estimated state as the one to be used
    usedX[0] = currentEstimatedX[0];
    usedX[1] = currentEstimatedX[1];
  }
  
  // Update difference between the manually estimated state and the observer estimated state
  estimatedXDiff[0] = currentEstimatedX[0] - currentXHat[0];
  estimatedXDiff[1] = currentEstimatedX[1] - currentXHat[1];

  // Implement control by updating small signal portion of PWM duty cycle
  if (controlOn && PWMDutyCycleLarge > 0) {
    // Calculate new small signal portion used to update total duty cycle
    PWMDutyCycleSmall = -1 * G[0] * usedX[0] - G[1] * usedX[1];
  }
  else {
    PWMDutyCycleSmall = 0;
  }

  // Convert small signal PWM from fractional value to actual value
  PWMDutyCycleSmall = PWMDutyCycleSmall * MAX_DUTY_CYCLE;

  // Add large and small signal components to get total updated duty cycle
  PWMDutyCycleTotal = PWMDutyCycleLarge + PWMDutyCycleSmall;

  // If limits are exceeded, if the PWM value is negative, or if the power is not enabled, cut motor power
  if (measuredTheta > PI/2.5 || measuredTheta < -PI/2.2 || PWMDutyCycleTotal < 0 || !powerOn){
    PWMDutyCycleTotal = 0;
  }
  // If max duty cycle is exceeded, set it to the max
  if (PWMDutyCycleTotal > MAX_DUTY_CYCLE){
    PWMDutyCycleTotal = MAX_DUTY_CYCLE;
  }
    
  // Set updated PWM output to desired duty cycle
  ledcWrite(PWMChannel, PWMDutyCycleTotal);

  // Check if zero point should be reset
  if (setZeroPoint) {
    myEncoder.write(0);   // Reset the current position as 0
  }

  // Update 6302view
  comManager.step();      // Includes explicit delay for STEP_TIME miliseconds
}
