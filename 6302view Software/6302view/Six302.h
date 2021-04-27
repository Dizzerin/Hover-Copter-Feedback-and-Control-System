
/* Reach out to almonds@mit.edu or jodalyst@mit.edu for help */

#ifndef _Six302_H_
#define _Six302_H_

/* CHOOSE ONE: */

#define S302_SERIAL
//#define S302_WEBSOCKETS

/* WebSockets options */

#define S302_PORT 80
#define S302_VERBOSE // enable this to print debug information to Serial (WebSockets)

#if ( defined S302_SERIAL &&  defined S302_WEBSOCKETS) || \
    (!defined S302_SERIAL && !defined S302_WEBSOCKETS)
#error "Choose a communication setup"
#endif

#if defined (S302_WEBSOCKETS) && !defined (ESP32) && !defined (ESP8266)
#error "Communication over WebSockets is only available for the ESP32 or ESP8266."
#endif

/* #includes */

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#if defined S302_WEBSOCKETS
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WebSocketsServer.h>
using namespace std::placeholders;
#endif

/* Sugar */

#if defined ESP32
#define TAKE xSemaphoreTake(_baton, portMAX_DELAY);
#define GIVE xSemaphoreGive(_baton);
#else
#define TAKE
#define GIVE
#endif

#if defined S302_SERIAL
#define BROADCAST(msg, len) _serial->write((uint8_t*)msg, len)
#elif defined S302_WEBSOCKETS
#define BROADCAST(msg, len) _wss.broadcastBIN((uint8_t*)msg, len)
#endif

#if defined (ESP32) || (ESP8266) || (TEENSYDUINO)
#else
#define S302_UNO
#endif

/* Memory constraints */

#if defined S302_UNO

// ARDUINO UNO:

         #define MAX_CONTROLS  5 // Joystick counts as two controls btw
         #define MAX_REPORTERS 5
         #define MAX_BURST     5

         #define MAX_TITLE_LEN 20
         #define MAX_DEBUG_LEN 500

         #define MAX_PREC      7

#else

// All else:

         #define MAX_CONTROLS  20
         #define MAX_REPORTERS 10
         #define MAX_BURST     10

         #define MAX_TITLE_LEN 30
         #define MAX_DEBUG_LEN 1000

#endif

// (conservative calculations:)
#define MAX_BUFFER_LEN (1+8+MAX_TITLE_LEN+24*5+5+1) // 145 last time checked
#define MAX_BUILD_STRING_LEN (2+MAX_CONTROLS*(8+MAX_TITLE_LEN+3*24+5)+1) // 1903 last time checked

/* Class definition! */

class CommManager {

   /* PUBLIC */

   public:

      /* Initialization */

      CommManager(uint32_t sp=1000, uint32_t rp=20000);
      
#if defined S302_SERIAL
#if defined TEENSYDUINO
      void connect(usb_serial_class* s, uint32_t baud);
#else
      void connect(HardwareSerial* s, uint32_t baud);
#endif
#elif defined S302_WEBSOCKETS
      void connect(char* ssid, char* pw);
#endif

      /* To add controls: */

      bool addToggle(
         bool* linker,
         const char* title);

      bool addButton(
         bool* linker,
         const char* title);

      bool addSlider(
         float* linker,
         const char* title,
         float range_min, float range_max,
         float resolution,
         bool toggle=false);

//      bool addJoystick(
//         float* linker_x,
//         float* linker_y,
//         const char* title,
//         float xrange_min, float xrange_max,
//         float yrange_min, float yrange_max,
//         float resolution,
//         bool sticky=true);

      /* To add reporters: */

      bool addPlot(
         float* linker,
         const char* title,
         float yrange_min, float yrange_max,
         uint8_t steps_displayed=10,
         uint8_t burst=1,
         uint8_t num_plots=1);

      bool addNumber(
         int32_t* linker,
         const char* title,
         uint8_t burst=1);

      bool addNumber(
         float* linker,
         const char* title,
         uint8_t burst=1);

      /* Tick */

      void step();
#if defined ESP32
      void _step();
#endif
      uint32_t headroom();

      /* Other */
      
      void debug(char*);
      void debug(String);
      //void debug(bool);
      //void debug(int);
      //void debug(float);
      //void debug(double);

   /* PRIVATE */

   protected:

      /* Most important buffers */

      char    _buf[MAX_BUFFER_LEN]; // long general buffer
      char    _build_string[MAX_BUILD_STRING_LEN];
      char    _debug_string[MAX_DEBUG_LEN];
      char    _tmp[24]; // short general buffer

      /* Burst mechanic */

      uint8_t _recordings[MAX_REPORTERS][MAX_BURST][4];
      uint8_t _bursts[MAX_REPORTERS];

      /* Links */

      float*  _controls[MAX_CONTROLS];   uint8_t _total_controls;
      float*  _reporters[MAX_REPORTERS]; uint8_t _total_reporters;

      /* Remember the inputs' types */
      bool _ctrl_types[MAX_CONTROLS];

#if defined S302_SERIAL
#if defined TEENSYDUINO
      usb_serial_class* _serial;
#else
      HardwareSerial* _serial;
#endif
      uint32_t _baud;
#elif defined S302_WEBSOCKETS
      WebSocketsServer _wss = WebSocketsServer(S302_PORT);
      void _on_websocket_event(
         uint8_t num, WStype_t type,
         uint8_t* payload, size_t length);
#endif
      
      /* Timing */

      uint32_t _step_period;
      uint32_t _report_period;
      int32_t  _headroom;    // headroom for the last step
      float    _headroom_rp; // lowest headroom over the last report period

#if defined TEENSYDUINO
      elapsedMicros _main_timer;
      elapsedMicros _report_timer;
#else
      uint32_t _main_timer;
      uint32_t _report_timer;
#endif
#if defined ESP32
      uint32_t _secondary_timer;
#endif

      /* Semaphore handle for the ESP32 */

#if defined ESP32
      SemaphoreHandle_t _baton;
#endif
      
      /* Routines */

      void _control();
      void _record(uint8_t reporter);
      void _report();
      bool _time_to_talk(uint32_t time_to_wait);
      void _wait();
      
      void _NOT_IMPLEMENTED_YET();

      // ESP32 dual core
#if defined ESP32
      TaskHandle_t _six302_task;
      static void _walk(void* param);
#endif

};

#endif