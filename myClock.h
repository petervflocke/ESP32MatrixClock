// Turn on debug statements to the serial output

#define  DEBUG_ON 1

#if  DEBUG_ON
#define PRINT(s, v) { Serial.print(s); Serial.print(v); }    
#define PRINTX(s, v) { Serial.print(s); Serial.print(v, HEX); }  
#define PRINTS(s) Serial.print(s)   
#define PRINTLN Serial.println()
#else
#define PRINT(s, v)   
#define PRINTX(s, v)  
#define PRINTS(s)     
#define PRINTLN
#define PRINTT 
#endif

// For system Pulse Indicator
#define ledPin 2
// for booting time to enable/disable wifi config
#define modePin 27
// MP3 pins
#define MP3RX 16
#define MP3TX 17

// set up encoder object and button
// MD_REncoder R = MD_REncoder(PIN_A, PIN_B);
#define PIN_A 13
#define PIN_B 15
#define PIN_BUT 12
#define SW_DOWN 'D'
#define SW_UP 'U'

// define number of objects/rotary & button events to be save in the keyboard queue
#define QUEUE_SIZE 15

// DAC out
#define DACOut     25
#define MaxV      255
#define MinV        5
#define VADelay    25
#define VADec       5

#define pirPin     14
#define DarkLevel  50
#define ScreenTimeOut 300

// PPMax72xxPanel definitions
// Attach CS to this pin, DIN to MOSI and CLK to SCK (cf http://arduino.cc/en/Reference/SPI )
#define pinCS 10
#define numberOfHorizontalDisplays 8
#define numberOfVerticalDisplays 1

// numberOfHorizontalDisplays*8
#define NumberOfPoints 56
#define ShiftDiagram 8
#define MeasurementFreg 1
// #define MaxMeasurements SECS_PER_DAY
#define MaxMeasurements SECS_PER_DAY
typedef float temp_t;
typedef float humi_t;
typedef float pres_t;

// PPmax72xxAnimate: Zone borders s- Start e- End for the matrix display
#define H1s  0
#define H1e  5
#define H0s  6
#define H0e 11
#define M1s 14
#define M1e 19
#define M0s 20
#define M0e 25
#define S1s 28
#define S1e 33
#define S0s 34
#define S0e 39
#define I0s  0
#define I0e 64
#define I1s 30
#define I1e 64
#define I2s  0
#define I2e 64
#define MEs  0
#define MEe 40
#define PAs 41
#define PAe 64

#define SNs 40
#define SNe 64
#define MaxSnake 64
#define SnakeAttempt 10
#define SankeNextRound 10
#define pinRandom 32
#define SnakeWait 50

#define ClockAnimTick 95
#define InfoTick  15
#define InfoTick1 25
#define InfoSlow  100
#define InfoQuick 18
#define FullInfoDelay 2000
#define DiagramDelay 200

#define IntensityWait 250

// Time zone definition and DST automatic on/off
// #define DST_ON true
#define CEST 2
#define CET  1
// Define time to resync
// 12h = 12*60*60=720
#define NTPRESYNC 43200
// define next to next hour for a hour chime 
#define CHIMEH 3600
// time of the 4. quorter chime in ms
#define CHIMEW 4200
// define next seconds to next qurter for a qurter chime 
#define CHIMEQ 900
#define DingON   9
#define DingOFF 23
// define freq. to update MQTT/IOT page in seconds
#define Time2UpdateMQTT 60

// IOT defines

#define feedTemp "/feeds/temperature"
#define feedHumi "/feeds/humidity"
#define feedPres "/feeds/pressure"
#define feedBrig "/feeds/brightness"
#define feedData "/feeds/data"
#define feedLED  "/feeds/led"

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883                   // use 8883 for SSL -1883

//#define AIO_USERNAME    this is defined in non-public file, define here for you
//#define AIO_KEY         this is defined in non-public file, define here for you
#include "credentials.h"



enum ClockStates
{
   _Clock_init,
   _Clock_NTP_Sync,
   _Clock_simple_time_init,
   _Clock_simple_time,
   _Clock_complete_info_init,
   _Clock_complete_info,
   _Clock_menu_init,
   _Clock_menu,
   _Clock_Temp_init,
   _Clock_Temp,   
   _Clock_idle,
   _Clock_none
};

enum DataStates
{
   _Data_Day,
   _Data_Month
};


enum SnakeStates_t
{
   _sInit,
   _sRunA,
   _sRunB,
   _sFail,
};

