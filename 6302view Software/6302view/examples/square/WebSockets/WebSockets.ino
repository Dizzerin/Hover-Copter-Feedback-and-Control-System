
#include <Six302.h>

/* For this demo, make sure to have the `#define S302_WEBSOCKETS`
   macro enabled in the library, and to use either an ESP32 or
   an ESP8266. */

// microseconds
#define STEP_TIME 100000
#define REPORT_TIME 500000

CommManager cm(STEP_TIME, REPORT_TIME);

float input;
float output;

void setup() {
   Serial.begin(115200);
   while(!Serial);
   
   /* Add modules */
   cm.addSlider(&input, "Input", -5, 5, 0.1);
   cm.addPlot(&output, "Output", -1, 30);

   /* Ready to communicate over websockets */
   cm.connect("MY NETWORK", "MY PASSWORD");
}

void loop() {
   output = input * input;
   cm.step();
}