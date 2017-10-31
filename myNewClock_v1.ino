/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include "myClock.h"  
#include <TimeLib.h>
#include <SPI.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include "Parola_fonts.h"
#include <WiFi.h>
#include <WiFiManager.h>   
#include <WiFiUdp.h>
#include <NTPClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 600000);
MD_Parola   P  = MD_Parola(CS_PIN, MAX_DEVICES);
boolean DstFlag = false; // indicate if DS is on/off based on the time for Germany / CET,CEST


ClockStates ClockState = _Clock_init;

// Buffers for clock digits
static char sH1[] = "0";
static char sH0[] = "0";
static char sM1[] = "0";
static char sM0[] = "0";
static char sS1[] = "0";
static char sS0[] = "0";
// current and last time values
static int valueH = 0;
static int valueM = 0;
static int valueS = 0;
static int lvalueH = -1;
static int lvalueM = -1;
static int lvalueS = -1;   

#define EFFECT 13
typedef struct
{
  textEffect_t  effect;   // text effect to display
  uint16_t      speed;    // speed multiplier of library default
  uint16_t      pause;    // pause multiplier for library default
} sCatalog;

sCatalog  catalog[] =
{
  { PA_PRINT,              1, 1 },    //  0
  { PA_SLICE,              1, 1 },    //  1
  { PA_MESH,              20, 1 },    //  2
  { PA_FADE,              20, 1 },    //  3
  { PA_WIPE,               5, 1 },    //  4
  { PA_WIPE_CURSOR,        4, 1 },    //  5
  { PA_OPENING,            3, 1 },    //  6
  { PA_OPENING_CURSOR,     4, 1 },    //  7
  { PA_CLOSING,            3, 1 },    //  8
  { PA_CLOSING_CURSOR,     4, 1 },    //  9
  { PA_RANDOM,             3, 1 },    // 10
  { PA_BLINDS,             1, 1 },    // 11
  { PA_DISSOLVE,           7, 1 },    // 12
  { PA_SCROLL_UP,          5, 1 },    // 13
  { PA_SCROLL_DOWN,        5, 1 },    // 14
  { PA_SCROLL_LEFT,        5, 1 },    // 15
  { PA_SCROLL_RIGHT,       5, 1 },    // 16
  { PA_SCROLL_UP_LEFT,     7, 1 },    // 17
  { PA_SCROLL_UP_RIGHT,    7, 1 },    // 18
  { PA_SCROLL_DOWN_LEFT,   7, 1 },    // 19
  { PA_SCROLL_DOWN_RIGHT,  7, 1 },    // 20
  { PA_SCAN_HORIZ,         4, 1 },    // 21
  { PA_SCAN_HORIZX,        4, 1 },    // 22
  { PA_SCAN_VERT,          3, 1 },    // 23
  { PA_SCAN_VERTX,         3, 1 },    // 24
  { PA_GROW_UP,            7, 1 },    // 25
  { PA_GROW_DOWN,          7, 1 },    // 26
};


int effectnr = 11;
bool newMessageAvailable = false;
String inString = "";

void readSerial(void)
{
  // Read serial input:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      effectnr = inString.toInt();
      newMessageAvailable = true;
      inString = "";

      if (effectnr == 0) {
        setTime(1509242390);
      }
      if (effectnr == 1) {

        if (WiFi.isConnected()) {
          SyncNTP();
        } else {
          Serial.print("Recovering connection\nStatus: ");
          WiFi.mode(WIFI_STA);
          WiFi.begin();
          delay(2000);
          SyncNTP();
          WiFi.mode(WIFI_OFF);
        }
      }
    }
  }
}




// Time management 
time_t requestSync() {return 0;} // the time will be sent later in response to serial mesg
void processSyncMessage() {
  // unsigned long pctime = 1509235190;
  unsigned long pctime = timeClient.getEpochTime();
  setTime(pctime); // Sync clock to the time received on the serial port
}
boolean IsDst(uint8_t _month, uint8_t _day, uint8_t _weekday) {
// Dst = True for summer time 
// _weekday = day of the week Sunday = 1, Saturday = 7

  if (_month < 3 || _month > 10)  return false; 
  if (_month > 3 && _month < 10)  return true; 

  int previousSunday = _day - _weekday;
  if (_month == 3)  return previousSunday >= 24;
  if (_month == 10) return previousSunday < 24;

  return false; // this line never gonna happend
}

boolean SyncNTP() {

  Serial.println("Starting NTP Sync\n");
  if (timeClient.forceUpdate()){
    setTime(timeClient.getEpochTime());
    Serial.print("NTP sync OK, UTC=");
    Serial.println(timeClient.getFormattedTime() );
    return true;
  } else {
    Serial.println("NTP sync failed!");
    return false;    
  }
}


boolean FirstSyncNTP() {

  P.displayZoneText(0, "NTP Sync", PA_LEFT, 20, 250, PA_SCROLL_LEFT, PA_NO_EFFECT);  
  while (!P.displayAnimate()) ;
  
  Serial.println("Starting 1st time NTP\n");
  timeClient.begin();
  Serial.println("connecting: \n");
  delay(100);
  while (!timeClient.forceUpdate()){
    Serial.print(".");
    P.displayZoneText(0, "-", PA_LEFT, 10, 10, PA_SCROLL_LEFT, PA_NO_EFFECT);  
    while (!P.displayAnimate()) ;
  }
  Serial.print("NTP sync OK, UTC=");
  Serial.println(timeClient.getFormattedTime() );
  P.displayZoneText(0, "Sync OK", PA_LEFT, 20, 250, PA_SCROLL_LEFT, PA_NO_EFFECT);  
  while (!P.displayAnimate()) ;
  

  setSyncProvider(requestSync);  //set function to call when sync required  
  processSyncMessage();

  // Check DST
  if (IsDst(month(), day(), weekday())) {
    DstFlag = true;
    timeClient.setTimeOffset(SECS_PER_HOUR*CEST);
    adjustTime(+SECS_PER_HOUR*CEST);
  } else {
    DstFlag = false;
    timeClient.setTimeOffset(SECS_PER_HOUR*CET);
    adjustTime(+SECS_PER_HOUR*CET);
  }    
  return true;
}


bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void SetupWiFi(void) {
    WiFiManager wifiManager; 
  
    //reset settings - for testing
    //wifiManager.resetSettings();
    P.displayZoneText(0, "WiFi Setup", PA_LEFT, SPEED_TIME, 1000, PA_SCROLL_LEFT, PA_NO_EFFECT);  
    while (!P.displayAnimate()) ;
    Serial.println("[Staring WiFi Manager]\n");
    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    //wifiManager.setAPCallback(configModeCallback);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setMinimumSignalQuality();
    // wifiManager.setAPStaticIPConfig(IPAddress(192,168,2,1), IPAddress(192,168,2,1), IPAddress(255,255,255,0));
    if (!wifiManager.autoConnect("ClockWiFi", "password")) {
      Serial.println("\nfailed to connect and hit timeout");
      delay(3000);
      // reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
    Serial.println("...WIFI connected!\n");
    P.displayZoneText(0, "Connected", PA_LEFT, SPEED_TIME, 1000, PA_SCROLL_LEFT, PA_NO_EFFECT);    
    while (!P.displayAnimate()) ;  
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

boolean update_time(int &value, int &lvalue, int newvalue, char * buf1, char* buf0, int zone1, int zone0) {
  
  
  
  if (value != newvalue)
  {
    value = newvalue;
    if ( lvalue / 10 != value / 10 ) {
      buf1[0] = (char)('0' + (value / 10));
      P.displayZoneText(zone1, buf1, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[effectnr].effect, PA_NO_EFFECT);
    }
    buf0[0] = (char)('0' + (value % 10));
    P.displayZoneText(zone0, buf0, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[effectnr].effect, PA_NO_EFFECT);
    lvalue = value;
    return true;
  }
  else return false;
}


void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println("[Clock started]\n");
    
    pinMode(ledPin, OUTPUT);

    P.begin(MAX_ZONES);
    P.setZone(0, 0, 7);
    P.setIntensity(0, 0);
    P.setFont(0, f5x8);
    
    SetupWiFi();
    FirstSyncNTP();

    WiFi.mode(WIFI_OFF);

    

    ClockState = _Clock_init;
     
}

void loop()
{
    static bool flasher = false;  // seconds passing flasher
    
    
    
  boolean nix;

  switch(ClockState)
  {
      case _Clock_init:

//          PRINTS("INIT CLosed\n");

          ClockState = _Clock_wifi_setup;
          break;
          
      case _Clock_wifi_setup:
         
          ClockState = _Clock_ntp_connect;
          break;
          
      case _Clock_ntp_connect:

          ClockState = _Clock_simple_time_init;
          break;
          
      case _Clock_ntp_update:
          
          break;
          
      case _Clock_simple_time_init:

          P.displayClear();

          P.setZone(H1, 6, 6); P.setIntensity(H1, 0); P.setFont(H1, f5x8);
          P.setZone(H0, 5, 5); P.setIntensity(H0, 0); P.setFont(H0, f5x8);
          P.setZone(M1, 4, 4); P.setIntensity(M1, 0); P.setFont(M1, f5x8);
          P.setZone(M0, 3, 3); P.setIntensity(M0, 0); P.setFont(M0, f5x8);
          P.setZone(S1, 2, 2); P.setIntensity(S1, 0); P.setFont(S1, f5x8);
          P.setZone(S0, 1, 1); P.setIntensity(S0, 0); P.setFont(S0, f5x8);
          P.setZone(RR, 0, 0); P.setIntensity(RR, 0); P.setFont(RR, f5x8);
        
          valueH = hour();
          valueM = minute();
          valueS = second();
          lvalueH = valueH;
          lvalueM = valueM;
          lvalueS = valueS;
        
          sH1[0] = (char)('0' + valueH / 10);
          sH0[0] = (char)('0' + (valueH % 10));
          sM1[0] = (char)('0' + (valueM / 10));
          sM0[0] = (char)('0' + (valueM % 10));  
          sS1[0] = (char)('0' + (valueS / 10));
          sS0[0] = (char)('0' + (valueS % 10));
        
          P.displayZoneText(H1, sH1, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[EFFECT].effect, PA_NO_EFFECT);
          P.displayZoneText(H0, sH0, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[EFFECT].effect, PA_NO_EFFECT);
          P.displayZoneText(M1, sM1, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[EFFECT].effect, PA_NO_EFFECT);
          P.displayZoneText(M0, sM0, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[EFFECT].effect, PA_NO_EFFECT);
          P.displayZoneText(S1, sS1, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[EFFECT].effect, PA_NO_EFFECT);
          P.displayZoneText(S0, sS0, PA_CENTER, SPEED_TIME, PAUSE_TIME, catalog[EFFECT].effect, PA_NO_EFFECT);

          delay(1000);
          Serial.println("Simple Time Init closed");
          
          ClockState = _Clock_simple_time;
          break;

      case _Clock_simple_time:
          update_time(valueH, lvalueH, hour(), sH1, sH0, H1, H0);
          update_time(valueM, lvalueM, minute(), sM1, sM0, M1, M0);
          nix = false;
          if ( update_time(valueS, lvalueS, second(), sS1, sS0, S1, S0) ) {
            flasher = !flasher;
            digitalWrite(ledPin, flasher);
            //Serial.println(timeClient.getFormattedTime() );
          }
          break;
      case _Clock_idle:
          
          break;

      default:;
  }

  if ( ClockState != _Clock_init &&
       ClockState != _Clock_wifi_setup &&
       ClockState != _Clock_ntp_connect) {
          //Check if DST has to correct the time
          if (month() == 3) {
            if (day() - weekday() >= 24) {
              if ( (hour() == 2) && (!DstFlag) ) {
                DstFlag = true;
                adjustTime(+SECS_PER_HOUR);
              }
            }
          } else if (month() == 10) {
            if (day() - weekday() >= 24) {
              if ( (hour() == 2) && (DstFlag) ) {
                DstFlag = false;
                adjustTime(-SECS_PER_HOUR);
              }      
            }
          }        
       }

  
  P.displayAnimate();
  P._D.setPoint(2,PW_secp, flasher);
  P._D.setPoint(5,PW_secp, flasher);
  P._D.setPoint(0,3*8-1, true);
  P._D.setPoint(1,3*8-1, true);

  readSerial();
  if (newMessageAvailable)
  {
    newMessageAvailable = false;
    for (int i = 0; i < MAX_ZONES; P.setTextEffect(i++, catalog[effectnr].effect, PA_NO_EFFECT));
  }
    

}

