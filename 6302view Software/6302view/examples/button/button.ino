
#include <Six302.h>

/* For this demo, make sure to have the `#define S302_SERIAL`
   macro enabled in the library. 

   This demo presents a button that when pressed increments
   the number displayed */

// microseconds
#define STEP_TIME 10000
#define REPORT_TIME 50000

CommManager cm(STEP_TIME, REPORT_TIME);

bool input;
int32_t output = 1;

void setup() {
   
   /* Add modules */
   cm.addButton(&input, "Increment");
   cm.addNumber(&output, "Sum");

   /* Ready to communicate over Serial */
   cm.connect(&Serial, 115200);
}

void loop() {
   if( input )
      output++;
   cm.step();
}