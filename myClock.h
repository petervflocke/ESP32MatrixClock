// Turn on debug statements to the serial output
#define  DEBUG  0

//#if  DEBUG
//#define PRINT(s, x) { Serial.print(F(s)); Serial.println(x); }
//#define PRINTS(x) Serial.print(F(x))
//#define PRINTX(x) Serial.println(x, HEX)
//#else
//#define PRINT(s, x)
//#define PRINTS(x)
//#define PRINTX(x)
//#endif

// For system Pulse Indicator
#define ledPin 2

//PPMax72xxPanel definitions
#define pinCS = 10; // Attach CS to this pin, DIN to MOSI and CLK to SCK (cf http://arduino.cc/en/Reference/SPI )
#define numberOfHorizontalDisplays = 8;
#define numberOfVerticalDisplays = 1;


// PPmax72xxAnimate: Zone numbers for the simple time display
#define H1s  0
#define H1e  6
#define H0s  6
#define H0e 12
#define M1s 15
#define M1e 20
#define M0s 20
#define M0e 25
#define S1s 28
#define S1e 32
#define S0s 32
#define S0e 37
#define I0s  0
#define I0e 64
#define I1s 27
#define I1e 64
#define I2s  0
#define I2e 64

// Old to be removed: Zone numbers for the simple time display

#define H1 0
#define H0 1
#define M1 2
#define M0 3
#define S1 4
#define S0 5
#define RR 6


// Time zone definition and DST automatic on/off
// #define DST_ON true
#define CEST 2
#define CET  1
// Define time to resync
// 12h
#define NTPRESYNC 43200
// 1h
#define NTPRESYNC_Err 3600

enum ClockStates
{
   _Clock_init,
   _Clock_wifi_setup,
   _Clock_NTP_Sync,
   _Clock_simple_time_init,
   _Clock_simple_time,
   _Clock_complete_info_init,
   _Clock_complete_info,
   _Clock_idle
};

enum DataStates
{
   _Data_Day,
   _Data_Month
};

