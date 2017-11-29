/*
 *  Under development, not produtive :)
 *
 *  NEW: workaround for parola issue and system restart for max zones > 7
 *  
 *
 */

#include "myClock.h"  
#include <TimeLib.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <PPMax72xxPanel.h>
#include <PPmax72xxAnimate.h>

PPMax72xxPanel matrix = PPMax72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);
PPmax72xxAnimate zoneClockH0 = PPmax72xxAnimate(&matrix);
PPmax72xxAnimate zoneClockH1 = PPmax72xxAnimate(&matrix);
PPmax72xxAnimate zoneClockM0 = PPmax72xxAnimate(&matrix);
PPmax72xxAnimate zoneClockM1 = PPmax72xxAnimate(&matrix);
PPmax72xxAnimate zoneClockS1 = PPmax72xxAnimate(&matrix);
PPmax72xxAnimate zoneClockS0 = PPmax72xxAnimate(&matrix);

PPmax72xxAnimate zoneInfo0 = PPmax72xxAnimate(&matrix);
PPmax72xxAnimate zoneInfo1 = PPmax72xxAnimate(&matrix);
//PPmax72xxAnimate zoneInfo2 = PPmax72xxAnimate(&matrix);


#include <WiFi.h>
#include <WiFiManager.h>   
#include <WiFiUdp.h>
#include <NTPClient.h>

#include <myScheduler.h>

#include <Wire.h>
#include <BH1750.h>
#include <SparkFunBME280.h>

BH1750 lightMeter;
BME280 mySensor;


#include <MD_CirQueue.h>
const uint8_t  QUEUE_SIZE = 15;
MD_CirQueue Q(QUEUE_SIZE, sizeof(uint8_t));

#include <MD_REncoder.h>
#define PIN_A 13
#define PIN_B 15

#define PIN_BUT 12
#define SW_DOWN 'D'
#define SW_UP 'U'

// set up encoder object
MD_REncoder R = MD_REncoder(PIN_A, PIN_B);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0);
unsigned long NTPSyncPeriod = NTPRESYNC;

boolean DstFlag = false; // indicate if DS is on/off based on the time for Germany / CET,CEST


ClockStates ClockState  = _Clock_init;  // current clock status
ClockStates lClockState = _Clock_init;  // last clock status

// current and last time values
static int valueH = 0;
static int valueM = 0;
static int valueS = 0;
static int lvalueH = -1;
static int lvalueM = -1;
static int lvalueS = -1;   

bool newMessageAvailable = false;
String inString = "";
void readSerial(void)  {
  int effectnr;
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
        ClockState = _Clock_simple_time_init;
        // setTime(1509242390);
        //P0.setIntensity(0);
        //P0.displayClear();
        //P0.displayZoneText(0, "De-Sync Clock", PA_LEFT, 25, 1000, PA_SCROLL_LEFT, PA_NO_EFFECT);  
        //while (!P0.displayAnimate());
        //P0.displayClear();

      }
      if (effectnr == 1) {
        ClockState = _Clock_complete_info_init;
//        if (WiFi.isConnected()) {  //rather not :)
//          SyncNTP();
//        } else {
//          Serial.println("Recovering WiFi connection:");
//          WiFi.mode(WIFI_STA);
//          WiFi.begin();
//          delay(2000);
//          SyncNTP();
//          WiFi.mode(WIFI_OFF);
//        }
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

  PRINTS("Starting NTP Sync\n");
  if (timeClient.forceUpdate()){
    setTime(timeClient.getEpochTime());
    PRINTS("NTP sync OK, UTC=");
    PRINTS(timeClient.getFormattedTime() ); 
    PRINTLN;
    return true;
  } else {
    PRINTS("NTP sync failed!");
    return false;    
  }
}

boolean FirstSyncNTP() {
  uint8_t x=0;
  zoneInfo0.setText("NTP Sync", _SCROLL_LEFT, _TO_LEFT, InfoTick, I0s, I0e);
  zoneInfo0.Animate(true);
    
  PRINTS("Starting 1st time NTP\n");
  timeClient.begin();
  PRINTS("connecting: \n");
  delay(100);
  while (!timeClient.forceUpdate()){
    PRINTS(".");
    matrix.drawPixel(x+45, 7, 1);
    x = (x +1) % 19;
    matrix.write();
    delay(250);
    matrix.drawPixel(x+44, 7, 1);
    matrix.write();
  }
  PRINTS("NTP sync OK, UTC=");
  PRINTS(timeClient.getFormattedTime() );
  PRINTLN;
  zoneInfo0.setText("Sync OK", _SCROLL_LEFT, _TO_LEFT, InfoTick, I0s, I0e);
  zoneInfo0.Animate(true);  

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
  PRINTS("WIFI saves config\n");
  shouldSaveConfig = true;
}

void SetupWiFi(void) {
    WiFiManager wifiManager; 
  
    //reset settings - for testing
    //wifiManager.resetSettings();
    zoneInfo0.setText("WiFi Setup", _SCROLL_LEFT, _TO_LEFT, InfoTick, I0s, I0e);
    zoneInfo0.Animate(true);
    PRINTS("Staring WiFi Manager\n");
    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    //wifiManager.setAPCallback(configModeCallback);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setMinimumSignalQuality();
    // wifiManager.setAPStaticIPConfig(IPAddress(192,168,2,1), IPAddress(192,168,2,1), IPAddress(255,255,255,0));
    if (!wifiManager.autoConnect("ClockWiFi", "password")) {
      PRINTS("\nfailed to connect and hit timeout");
      delay(3000);
      // reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
    PRINTS("...WIFI connected!\n");
    zoneInfo0.setText("Connected", _SCROLL_LEFT, _TO_LEFT, InfoTick, I0s, I0e);
    zoneInfo0.Animate(true);
    PRINTS("WiFi connected\n");
    PRINTS("IP address: \n");
    PRINTS(WiFi.localIP());
}

boolean update_time(int &value, int &lvalue, int newvalue, char what) {
  String sValue;
  if (value != newvalue)  {
    value = newvalue;
    if ( lvalue / 10 != value / 10 ) {
      sValue = String(lvalue /10) + "\n" + String(value /10);
      switch (what) {
        case 'H': zoneClockH1.setText(sValue, _SCROLL_UP_SMOOTH, _NONE_MOD, ClockAnimTick, H1s, H1e);
          break;
        case 'M': zoneClockM1.setText(sValue, _SCROLL_UP_SMOOTH, _NONE_MOD, ClockAnimTick, M1s, M1e);
          break;
        case 'S': zoneClockS1.setText(sValue, _SCROLL_UP_SMOOTH, _NONE_MOD, ClockAnimTick, S1s, S1e);
          break;
      }
    }
    sValue = String(lvalue % 10) + "\n" + String(value % 10); 
    switch (what) {
      case 'H': zoneClockH0.setText(sValue, _SCROLL_UP_SMOOTH, _NONE_MOD, ClockAnimTick, H0s, H0e);
        break;
      case 'M': zoneClockM0.setText(sValue, _SCROLL_UP_SMOOTH, _NONE_MOD, ClockAnimTick, M0s, M0e);
        break;
      case 'S': zoneClockS0.setText(sValue, _SCROLL_UP_SMOOTH, _NONE_MOD, ClockAnimTick, S0s, S0e);
        break;
    }
    lvalue = value;
    return true;
  } else return false;
}

void Status2Clock_NTP_Sync(void) {
  lClockState = ClockState;
  ClockState  = _Clock_NTP_Sync;
}


uint8_t IntensityMap(uint16_t sensor) {
   uint8_t Intensity;
   if (sensor < 80) Intensity = 0;
   else if (sensor < 110) Intensity = 1;
   else if (sensor < 140) Intensity = 2;
   else if (sensor < 170) Intensity = 3;
   else if (sensor < 200) Intensity = 4;
   else if (sensor < 250) Intensity = 5;
   else if (sensor < 300) Intensity = 6;
   else if (sensor < 350) Intensity = 7;
   else if (sensor < 400) Intensity = 8;
   else if (sensor < 450) Intensity = 9;
   else if (sensor < 500) Intensity = 10;
   else if (sensor < 550) Intensity = 11;
   else if (sensor < 600) Intensity = 12;
   else if (sensor < 650) Intensity = 13;
   else if (sensor < 700) Intensity = 14;
   else Intensity = 15;
   return Intensity;  
}

int8_t keyboard(void) {
  uint32_t  n;
  if (!Q.isEmpty()) {
    Q.pop((uint8_t *)&n);
    switch(n) {
      case DIR_CW:
        Serial.println("+1");
        break;
      case DIR_CCW:
        Serial.println("-1");
        break;
      case SW_DOWN:
        Serial.println("DOWN");
        break;
      case SW_UP:
        Serial.println("UP");
        break;
    }  
  }
  
}


Schedular NTPUpdateTask; 
Schedular DataDisplayTask; 
Schedular IntensityCheck; 

void setup()
{
 //   #if  DEBUG_ON
      Serial.begin(115200);
      PRINTS("Clock started\n");
//    #endif 
    delay(100);    

    pinMode(ledPin,  OUTPUT); // passing seconds LED
    pinMode(modePin, INPUT_PULLUP); // check setup
         
    matrix.setIntensity(4); // Use a value between 0 and 15 for brightness
  
    matrix.setPosition(0, 7, 0); matrix.setRotation(0, 2);    
    matrix.setPosition(1, 6, 0); matrix.setRotation(1, 2);
    matrix.setPosition(2, 5, 0); matrix.setRotation(2, 2);
    matrix.setPosition(3, 4, 0); matrix.setRotation(3, 2);
    matrix.setPosition(4, 3, 0); matrix.setRotation(4, 2);
    matrix.setPosition(5, 2, 0); matrix.setRotation(5, 2);
    matrix.setPosition(6, 1, 0); matrix.setRotation(6, 2);
    matrix.setPosition(7, 0, 0); matrix.setRotation(7, 2);

    matrix.fillScreen(LOW);
    matrix.setCursor(0,0);
    //matrix.setTextColor(HIGH);
    matrix.setTextColor(HIGH, LOW);
    matrix.setTextWrap(false);

    //WiFi.mode(WIFI_OFF);

    Q.begin(); // start queue to collect button and rotery events

    pinMode(PIN_BUT, INPUT_PULLUP); // button to operate menu
    attachInterrupt(digitalPinToInterrupt(PIN_BUT), processButton, CHANGE);
     
    R.begin(); // start rotary decoder
    attachInterrupt(digitalPinToInterrupt(PIN_A), processEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), processEncoder, CHANGE);
    
    // SetUp Sensors
    Wire.begin(21,22, 400000);
    //  ADDR=0 => 0x23 and ADDR=1 => 0x5C.
    lightMeter.begin();
  
    mySensor.settings.commInterface = I2C_MODE;
    mySensor.settings.I2CAddress = 0x76;
    mySensor.settings.tStandby = 0;
    mySensor.settings.runMode = 3; //Normal mode
    mySensor.settings.filter = 0;
    mySensor.settings.tempOverSample = 1;
    mySensor.settings.pressOverSample = 1;
    mySensor.settings.humidOverSample = 1;
    mySensor.begin();
    
    matrix.setIntensity(IntensityMap(lightMeter.readLightLevel())); 
    zoneInfo0.setText("Starting Clock ...", _SCROLL_LEFT, _TO_FULL, 12, I0s, I0e);
    zoneInfo0.Animate(true);
    if (digitalRead(modePin)) {
      SetupWiFi();
      FirstSyncNTP();
      NTPUpdateTask.start(nextMidnight( now()*1000 + 5000)); 
      PRINT("Time to sync: ",numberOfMinutes ( nextMidnight( now() ) ) );
      PRINTLN;
    } else {
      PRINTS("No WiFi by setup on GPIO27\n"); 
    }
    
    matrix.setClip(0, matrix.width(), 0, matrix.height());
    matrix.fillScreen(LOW);
    matrix.write();    
    
    ClockState = _Clock_complete_info_init;
    //ClockState = _Clock_simple_time_init;
}

void loop()
{
    static bool flasher = false;          // seconds passing flasher
    static uint8_t intensity = 0;         // brithness of the led matrix - all modules
    static uint8_t lintensity = 0;        // last brithness of the led matrix - all modules
    static boolean updateDisplay = false; // Any change => display needs to be updated

    static char DataStr[] = "xx:xx:xx Xxx xx Xxx xxxx                ";
                          // 0123456789012345678901234567891234567890
                          // 0         2         3         4        5
                          // 23:59:59 Sun 31 Oct 2016 100°C 1000HPa
                          
    static uint8_t DataMode = 0;

    boolean SyncNTPError = false;  // true if the last NTP was finished with an error due the wifi or ntp failure


  if (IntensityCheck.check(250)) {
    uint16_t lux = lightMeter.readLightLevel();
    intensity = IntensityMap(lux);
    //PRINT("Light: ",lux);
    //PRINT(" lx  MAP:", intensity);
    //PRINTLN;
    if (intensity != lintensity) {
      //matrix.setIntensity(intensity);
      lintensity = intensity;
    }
  }
  
  switch(ClockState)
  {
      case _Clock_init:
          ClockState = _Clock_simple_time_init;
          break;
          
      case _Clock_NTP_Sync:
        zoneInfo0.setText("NTP Re-Sync", _SCROLL_LEFT, _TO_FULL, 50, I0s, I0e);
        zoneInfo0.Animate(true);
        if (WiFi.isConnected()) {  
            SyncNTPError = SyncNTP();
        } else {
            PRINTS("Recovering WiFi connection\n");
            //WiFi.mode(WIFI_STA);
            matrix.fillScreen(LOW);
            matrix.setCursor(0,0);
            zoneInfo0.setText("Connecting WiFi", _SCROLL_LEFT, _TO_FULL, 50, I0s, I0e);
            WiFi.begin();
            zoneInfo0.Animate(true);
            if (WiFi.isConnected()) {
              zoneInfo0.setText("Connecting NTP", _SCROLL_LEFT, _TO_FULL, 50, I0s, I0e);
              SyncNTPError = SyncNTP();
              zoneInfo0.Animate(true);
              //WiFi.mode(WIFI_OFF);
            } else {
              SyncNTPError = true; // wifi failure
            }
        }
        ClockState = lClockState;
        break;
                    
        case _Clock_simple_time_init:

          valueH = hour();
          valueM = minute();
          valueS = second();
          lvalueH = valueH;
          lvalueM = valueM;
          lvalueS = valueS;

          matrix.drawChar(H1s,0, (char)('0' + valueH / 10), HIGH, LOW, 1);
          matrix.drawChar(H0s,0, (char)('0' + valueH % 10), HIGH, LOW, 1);
          matrix.drawChar(M1s,0, (char)('0' + valueM / 10), HIGH, LOW, 1);
          matrix.drawChar(M0s,0, (char)('0' + valueM % 10), HIGH, LOW, 1);
          matrix.drawChar(S1s,0, (char)('0' + valueS / 10), HIGH, LOW, 1);
          matrix.drawChar(S0s,0, (char)('0' + valueS % 10), HIGH, LOW, 1);
          matrix.drawPixel(M0e+1,0,HIGH);
          matrix.drawPixel(M0e+1,1,HIGH);

          updateDisplay = true;

          IntensityCheck.start();
          
          PRINTS("Simple Time Init closed\n");
          
          ClockState = _Clock_simple_time;
          break;

      case _Clock_simple_time:
          update_time(valueH, lvalueH, hour(), 'H');
          update_time(valueM, lvalueM, minute(), 'M');
          if ( update_time(valueS, lvalueS, second(), 'S') ) {
            flasher = !flasher;
            digitalWrite(ledPin, flasher);
            updateDisplay = true;
            PRINTS(timeClient.getFormattedTime() );
            PRINTLN;
            matrix.setClip(H0e+1,H0e+2,0,8);
            matrix.drawPixel(H0e+1,2,flasher);
            matrix.drawPixel(H0e+1,5,flasher);
          }
          updateDisplay |= zoneClockH0.Animate(false);
          updateDisplay |= zoneClockH1.Animate(false);
          updateDisplay |= zoneClockM0.Animate(false);
          updateDisplay |= zoneClockM1.Animate(false);
          updateDisplay |= zoneClockS1.Animate(false); 
          updateDisplay |= zoneClockS0.Animate(false);

          break;

      case _Clock_complete_info_init:
       
          valueH = hour();
          valueM = minute();
          valueS = second();
          lvalueH = valueH;
          lvalueM = valueM;
          lvalueS = valueS;

          matrix.drawChar(H1s,0, (char)('0' + valueH / 10), HIGH, LOW, 1);
          matrix.drawChar(H0s,0, (char)('0' + valueH % 10), HIGH, LOW, 1);
          matrix.drawChar(M1s,0, (char)('0' + valueM / 10), HIGH, LOW, 1);
          matrix.drawChar(M0s,0, (char)('0' + valueM % 10), HIGH, LOW, 1);

          DataDisplayTask.start();
          IntensityCheck.start();
          
          PRINTS("Complete Time Init closed\n");
          
          ClockState = _Clock_complete_info;
          break;

      case _Clock_complete_info:
     
          update_time(valueH, lvalueH, hour(), 'H');
          update_time(valueM, lvalueM, minute(), 'M');
          if ( update_time(valueS, lvalueS, second(), 'S') ) {
            flasher = !flasher;
            digitalWrite(ledPin, flasher);
            updateDisplay = true;
            PRINTS(timeClient.getFormattedTime() );
            PRINTLN;
            matrix.setClip(H0e+1,H0e+2,0,8);
            matrix.drawPixel(H0e+1,2,flasher);
            matrix.drawPixel(H0e+1,5,flasher);
          }
          updateDisplay |= zoneClockH0.Animate(false);
          updateDisplay |= zoneClockH1.Animate(false);
          updateDisplay |= zoneClockM0.Animate(false);
          updateDisplay |= zoneClockM1.Animate(false);

            if ( DataDisplayTask.check(2000) ) {
              switch (DataMode){
                case 0:
                   sprintf (DataStr, "%02d%s", day(), monthShortStr(month()));
                   break;
                case 1:
                   sprintf (DataStr, "%s", dayStr(weekday()));
                   break;
                case 2: 
                   sprintf (DataStr, "%s", monthStr(month()));
                   break;
                case 3: 
                   sprintf (DataStr, "T:%d%s", (int)(mySensor.readTempC()+0.5), "C");
                   break;
                case 4: 
                   sprintf (DataStr, "C:%.0f%s", mySensor.readFloatPressure()/100, "HPa");
                   break;                   
                case 5: 
                   sprintf (DataStr, "W:%d%%", (int)(mySensor.readFloatHumidity()+0.5));
                   break;                   
                case 6: 
                   int measurement = hallRead();
                   PRINT("Hall sensor measurement: ", measurement);
                   sprintf (DataStr, "H:%02d", measurement);
                   break;                   
              }
              PRINTS(DataStr);
              PRINTLN;
              //zoneInfo1.setText(DataStr, _SCROLL_LEFT, _TO_LEFT, InfoTick1, I1s, I1e);
              zoneInfo1.setText(DataStr, _SCROLL_UP, _NONE_MOD, InfoTick1, I1s, I1e);
              DataMode = (DataMode+1) % 7;
            }
            updateDisplay |= zoneInfo1.Animate(false);
          break;          
          
      case _Clock_idle:
          
          break;

      default:;
  }

  if ( ClockState != _Clock_init ) {
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

  if (updateDisplay) {
    matrix.write();
    updateDisplay = false;
  }

  readSerial();
  if (newMessageAvailable)  {
    newMessageAvailable = false;
    // for (int i = 0; i < MAX_ZONES; P1.setTextEffect(i++, catalog[effectnr].effect, PA_NO_EFFECT));
  }
  
  NTPUpdateTask.check(Status2Clock_NTP_Sync, NTPSyncPeriod);

}

void processButton() {
   static volatile uint8_t SWPrev = 1;

   uint8_t state =  digitalRead(PIN_BUT);
   uint8_t x;
   if (state == 0) {
      if (SWPrev == 1) {
        SWPrev = 0;
        x = SW_DOWN;
        Q.push((uint8_t *)&x);
        // Serial.print("\nDown");
      } 
   } else { 
      if (SWPrev == 0) {
        SWPrev = 1;
        x = SW_UP;
        Q.push((uint8_t *)&x);
        // Serial.print("\nUp");
      } 
   }
}

void processEncoder () 
{
  uint8_t x = R.read();
  if (x) 
  {
    PRINTS((x == DIR_CW ? "\n+1" : "\n-1"));
    PRINTLN;
    Q.push((uint8_t *)&x);
  }
}

