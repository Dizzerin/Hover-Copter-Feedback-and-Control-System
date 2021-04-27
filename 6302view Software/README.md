# 6302view

**6302view** is a setup that allows researchers and learners to interact directly with their microcontroller in a web browser. Used in MIT subject 6.302 (Feedback system design).

Our goal of this project is to make it elegant -- just include the library and go -- no explicit need to modify anything depending on the microcontroller, for example.

[![(system setup)](https://i.imgur.com/djGt0lU.jpg "6302view with Teensy setup")](https://www.youtube.com/watch?v=AaNXcUNaw-I)

## Supported microcontrollers

* [Teensy 3.2](https://www.pjrc.com/teensy/)
* [ESP8266](https://en.wikipedia.org/wiki/ESP8266)
* [ESP32](https://www.espressif.com/en/products/hardware/esp32/overview)
* [Arduino Uno](https://www.arduino.cc/en/guide/BoardAnatomy)

## Quick example

```cpp

#include <Six302.h>

// microseconds
#define STEP_TIME 5000
#define REPORT_TIME 50000

CommManager cm(STEP_TIME, REPORT_TIME);

float input;
float output;

void setup() {
   /* Add modules */
   cm.addSlider(&input, "Input", 0, 5, 0.01);
   cm.addPlot(&output, "Output", -1, 26);

   /* Ready to communicate over serial */
   cm.connect(&Serial, 115200);
}

void loop() {
   output = input * input;
   cm.step();
}
```

The above creates a **Slider** called "Input" from 0 to 5 (at a step size of 0.01) and a **Plot** called "Output" from -1 to 26, illustrating the square of the slider input:

![(gif of resulting modules)](https://i.imgur.com/THO1Me1.gif)

The system loops at once per 5000 µs (5 ms), and data is reported once per 50000 µs (50 ms).

## Quick install

1. [Download this repo](https://github.com/almonds0166/6302view/archive/master.zip) and extract the contents to an unzipped folder.

2. Copy the `6302view/` folder that contains the C/C++ library to your [Arduino libraries folder](https://www.arduino.cc/en/guide/libraries).
   * On Windows and Mac, the folder is usually named `libraries/` and is located in `Arduino/` in your Documents folder.
   * On Linux, the folder is usually named `Sketchbook/` and located in `/home/`.

3. Install the Python dependencies `websockets` and `pyserial` using `pip`.

To run the local server, run `local_server.py`, located in `gui/`, with a Python version of at least 3.5. Finally, open `gui.html` in your favorite web browser.

## Dependencies

### Python modules used

*Only required for communication over Serial.*

```plaintext
websockets
pyserial
```

### Arduino libraries used

*Only required if you want to use the WiFi modules of ESP8266 or ESP32 as opposed to communication over Serial.*

Go to `Manage libraries...` and search for `WebSockets`. It's [the one at the bottom by Markus Sattler](https://github.com/Links2004/arduinoWebSockets).

## Documentation

[Documentation is in `docs.md`.](https://github.com/almonds0166/6302view/blob/master/docs.md)