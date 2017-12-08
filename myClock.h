// Turn on debug statements to the serial output

#define  DEBUG_ON 1

#if  DEBUG_ON
#define PRINT(s, v) { Serial.print(s); Serial.print(v); }    
#define PRINTX(s, v) { Serial.print(s); Serial.print(v, HEX); }  
#define PRINTS(s) Serial.print(s)   
#define PRINTLN Serial.println()
#define PRINTT digitalClockDisplay()
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

// PPMax72xxPanel definitions
// Attach CS to this pin, DIN to MOSI and CLK to SCK (cf http://arduino.cc/en/Reference/SPI )
#define pinCS 10
#define numberOfHorizontalDisplays 8
#define numberOfVerticalDisplays 1

// numberOfHorizontalDisplays*8
#define NumberOfPoints 64
#define MeasurementFreg 1
// #define MaxMeasurements SECS_PER_DAY
#define MaxMeasurements 10
#define PrecTemp 10
typedef float temp_T;

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
#define IntensityWait 250

// Time zone definition and DST automatic on/off
// #define DST_ON true
#define CEST 2
#define CET  1
// Define time to resync
// 12h = 12*60*60=43200
#define NTPRESYNC 43200

enum ClockStates
{
   _Clock_init,
   _Clock_NTP_Sync,
   _Clock_simple_time_init,
   _Clock_simple_time,
   _Clock_complete_info_init,
   _Clock_complete_info,
   _Clock_alarm_init,
   _Clock_alarm,
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


