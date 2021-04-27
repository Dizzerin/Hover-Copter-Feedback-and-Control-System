
# Demos

Nearly all demos expect to send information over Serial instead of WebSockets. The `square` demo has two versions, one for each communication method.

## `square`

There is one control (input) and one reporter (output). The input is between -5 and 5, and the output is the square of the input. There is one file for sending information over WebSockets and one for Serial.

## `button`

There is a button that increments a numerical display each press.

## `slider_toggle`

This demo tests the toggle functionality of sliders. It has one slider and one plot that should toggle between -1 and +1.

<!-- ## `joystick`

This demo tests the functionality of the joystick. In a plot and numerical display, it displays the distance and angle of the joystick vector. -->