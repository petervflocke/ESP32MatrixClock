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

//MD_Parola params:
#define MAX_ZONES 6
#define MAX_DEVICES 8
#define CLK_PIN   13
#define DATA_PIN  11
#define CS_PIN    10
#define PW_hour   52
#define PW_min    38
#define PW_secp   39
#define SPEED_TIME  25
#define PAUSE_TIME  1

// Zone numbers for the simple time display
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

