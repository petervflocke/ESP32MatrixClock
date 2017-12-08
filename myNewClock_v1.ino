/*
 *  Under development, somehow produtive :)
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

Schedular SensorUpdate(_Seconds); 
temp_T tempTable[NumberOfPoints];


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
boolean DSTFlag = false; // indicate if DS is on/off based on the time for Germany / CET,CEST
boolean SyncNTPError = false;  // true if the last NTP was finished with an error due the wifi or ntp failure

ClockStates ClockState  = _Clock_init;  // current clock status
ClockStates goBackState = _Clock_init;  // last clock status

void clearScreen(void) {
  matrix.setClip(0, matrix.width(), 0, matrix.height());
  matrix.fillScreen(LOW);
  matrix.write();    
}


// Time management 
time_t requestSync() {return 0;} // the time will be sent later in response to serial mesg

void processSyncMessage() {
  // unsigned long pctime = 1509235190;
  unsigned long pctime = timeClient.getEpochTime();
  setTime(pctime); // Sync clock to the time received on the serial port
}

boolean SetUpDST(uint8_t _month, uint8_t _day, uint8_t _weekday) {
// Dst = True for summer time 
// _weekday = day of the week Sunday = 1, Saturday = 7

  if (_month < 3 || _month > 10)  return false; 
  if (_month > 3 && _month < 10)  return true; 

  int previousSunday = _day - _weekday;
  if (_month == 3)  return previousSunday >= 24;
  if (_month == 10) return previousSunday < 24;

  return false; // this line never gonna happend
}

void correctByDST() {
  //Check if DST has to correct the time
  if (month() == 3) {
    if (day() - weekday() >= 24) {
      if ( (hour() == 2) && (!DSTFlag) ) {
        DSTFlag = true;
        adjustTime(+SECS_PER_HOUR);
      }
    }
  } else if (month() == 10) {
    if (day() - weekday() >= 24) {
      if ( (hour() == 2) && (DSTFlag) ) {
        DSTFlag = false;
        adjustTime(-SECS_PER_HOUR);
      }      
    }
  }        
}

boolean SyncNTP() {

  PRINTS("Contacting NTP Server process\n");
  if (timeClient.forceUpdate()){
    PRINTS("NTP sync OK, UTC="); PRINTS(timeClient.getFormattedTime() ); PRINTLN;
    setTime(timeClient.getEpochTime());
    return true;
  } else {
    PRINTS("NTP sync failed!");
    return false;    
  }
}

boolean StartSyncNTP() {
  boolean SyncError = false;
    
    PRINTS("NTP Sync Init started\n");
    zoneInfo0.setText("NTP Sync", _SCROLL_LEFT, _TO_LEFT, InfoTick1, I0s, I0e);
    zoneInfo0.Animate(true);
    if (WiFi.isConnected()) {  
        SyncError = !SyncNTP();
    } else {
        PRINTS("Recovering WiFi connection\n");
        //WiFi.mode(WIFI_STA);
        matrix.fillScreen(LOW);
        matrix.setCursor(0,0);
        zoneInfo0.setText("Connecting WiFi", _SCROLL_LEFT, _TO_FULL, InfoTick1, I0s, I0e);
        WiFi.begin();
        zoneInfo0.Animate(true);
        if (WiFi.isConnected()) {
          zoneInfo0.setText("Connecting NTP", _SCROLL_LEFT, _TO_FULL, InfoTick1, I0s, I0e);
          SyncError = !SyncNTP();
          zoneInfo0.Animate(true);
          //WiFi.mode(WIFI_OFF);
        } else {
          SyncError = true; // wifi failure
        }
    }    
  return SyncError;
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
    zoneInfo0.setText("WiFi Setup", _SCROLL_LEFT, _TO_LEFT, InfoTick1, I0s, I0e);
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
    zoneInfo0.setText("Connected", _SCROLL_LEFT, _TO_LEFT, InfoTick1, I0s, I0e);
    zoneInfo0.Animate(true);
    PRINTS("WiFi connected\n");
    PRINTS("IP address: \n");
    PRINTS(WiFi.localIP());
    PRINTLN;
}

boolean updateTtime(int &value, int &lvalue, int newvalue, char what) {
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

boolean checkTime(uint8_t hh, uint8_t mm, uint8_t ss, boolean exact) {
  if (exact) return (hh == hour() && mm == minute() && ss == second());
  else return (hh == hour() && mm == minute() && ss <= second());
}


uint8_t keyboard(ClockStates key_DIR_CW, ClockStates key_DIR_CCW, ClockStates key_SW_DOWN, ClockStates key_SW_UP) {
  uint8_t  key=DIR_NONE;
  noInterrupts();
  if (!Q.isEmpty()) {
    Q.pop(&key);
  }
  interrupts();

  switch(key) {
    case DIR_CW:
      if (key_DIR_CW != _Clock_none) ClockState = key_DIR_CW;
      break;
    case DIR_CCW:
      if (key_DIR_CCW != _Clock_none) ClockState = key_DIR_CCW;
      break;
    case SW_DOWN:
      if (key_SW_DOWN != _Clock_none) ClockState = key_SW_DOWN;
      break;
    case SW_UP:
      if (key_SW_UP != _Clock_none) ClockState = key_SW_UP;
      break;
  }
  #if DEBUG_ON
    if (key != DIR_NONE) {PRINT("New Closk State: ", ClockState); PRINTLN;}
  #endif
  return key;  
}

/* Snake Routines 
Start => */
boolean occupied(int ptrA, const int snakeLength, int *snakeX, int *snakeY) {
  for ( int ptrB = 0 ; ptrB < snakeLength; ptrB++ ) {
    if ( ptrA != ptrB ) {
      if ( equal(ptrA, ptrB, snakeX, snakeY) ) {
        return true;
      }
    }
  }
  return false;
}

int next(int ptr, const int snakeLength) {
  return (ptr + 1) % snakeLength;
}

boolean equal(int ptrA, int ptrB, int *snakeX, int *snakeY) {
  return snakeX[ptrA] == snakeX[ptrB] && snakeY[ptrA] == snakeY[ptrB];
}
/* <= End Snake Routines */

Schedular NTPUpdateTask(_Seconds); 
Schedular DataDisplayTask(_Millis); 
Schedular IntensityCheck(_Millis); 
Schedular SnakeUpdate(_Millis);
Schedular StatTask(_Millis); 


void setup()
{
 //   #if  DEBUG_ON
      Serial.begin(115200);
      PRINTS("Clock started\n");
//    #endif 
    delay(100);    

    pinMode(ledPin,  OUTPUT); // passing seconds LED
    pinMode(modePin, INPUT_PULLUP); // check setup
         
    matrix.setIntensity(0); // Use a value between 0 and 15 for brightness
  
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
    SensorUpdate.start(-1);

    // zero temperature table
    for (int i=0; i < NumberOfPoints; tempTable[i++]=0);
    IntensityCheck.start();
    matrix.setIntensity(IntensityMap(lightMeter.readLightLevel())); 
    zoneInfo0.setText("Starting Clock ...", _SCROLL_LEFT, _TO_LEFT, 12, I0s, I0e);
    zoneInfo0.Animate(true);
    if (digitalRead(modePin)) {
      SetupWiFi();
      SyncNTPError = StartSyncNTP();
      if (!SyncNTPError) {
        if (SetUpDST(month(), day(), weekday())) {
          DSTFlag = true;
          timeClient.setTimeOffset(SECS_PER_HOUR*CEST);
          adjustTime(+SECS_PER_HOUR*CEST);
        } else {
          DSTFlag = false;
          timeClient.setTimeOffset(SECS_PER_HOUR*CET);
          adjustTime(+SECS_PER_HOUR*CET);
        }    
        PRINTS("             Local Time="); PRINTS(timeClient.getFormattedTime() ); PRINTLN;
        correctByDST();
  //      PRINT("Now: ", now()); PRINTLN;
  //      PRINT("Hour: ", hour()); PRINTLN;
  //      PRINT("Min: ", minute()); PRINTLN;
  //      PRINT("Sec: ", second()); PRINTLN;
  //      PRINT("hour()%12): ", (hour()%12)); PRINTLN;
  //      PRINT("11-(hour()%12): ", 11-(hour()%12)); PRINTLN;
  //      PRINT("(11-(hour()%12))*3600: ", (11-hour()%12)*SECS_PER_HOUR); PRINTLN;
  //      PRINT("(60-minute())*60: ", (60-minute())*60); PRINTLN;
        // sync every 12 hours at 12:05 or 24:05 
        PRINT("Seconds to sync: ", (11-(hour()%12))*SECS_PER_HOUR+(60-minute()+10)*SECS_PER_MIN+60-second()); PRINTLN;
        NTPUpdateTask.start( (11-(hour()%12))*SECS_PER_HOUR+(60-minute()+10)*SECS_PER_MIN+60-second() );
        zoneInfo0.setText("Sync OK", _SCROLL_LEFT, _TO_LEFT, InfoTick1, I0s, I0e);
        zoneInfo0.Animate(true);
        delay(250);
      } else {
        PRINTS("Error at first NTP Sync\n");
        zoneInfo0.setText("1.NTP Sync ERROR", _SCROLL_LEFT, _TO_LEFT, InfoTick1, I0s, I0e);
        zoneInfo0.Animate(true);        
      }
    } else {
      PRINTS("No WiFi by setup on GPIO27\n"); 
      zoneInfo0.setText("WiFi Off", _SCROLL_LEFT, _TO_LEFT, InfoTick1, I0s, I0e);
      zoneInfo0.Animate(true);
    }
    matrix.setClip(0, matrix.width(), 0, matrix.height());
    matrix.fillScreen(LOW);
    matrix.write();    
    
    ClockState = _Clock_complete_info_init;
    //ClockState = _Clock_simple_time_init;
}

void loop()
{
    // current and last time values
    static int valueH = 0;
    static int valueM = 0;
    static int valueS = 0;
    static int lvalueH = -1;
    static int lvalueM = -1;
    static int lvalueS = -1;   
    
    
    static bool flasher = false;          // seconds passing flasher
    static uint8_t intensity = 0;         // brithness of the led matrix - all modules
    static uint8_t lintensity = 0;        // last brithness of the led matrix - all modules
    static boolean updateDisplay = false; // Any change => display needs to be updated

    static char DataStr[] = "xx:xx:xx Xxx xx Xxx xxxx                ";
                          // 0123456789012345678901234567891234567890
                          // 0         2         3         4        5
                          // 23:59:59 Sun 31 Oct 2016 100°C 1000HPa
                          
    static uint8_t DataMode = 0;
    uint8_t key;

    // index for tables with measurements
    static uint8_t dayNumber = 0 ; 
    static unsigned long measurementNumber = 1;
    static temp_T tempMin = +10000;
    static temp_T tempMax = -10000;
    temp_T tmpValue, tmpValue2;
    textEffect_t textEffect;
    

//    int16_t  tx1, ty1;
//    uint16_t tw, th;    
//    boolean decPoint;

    // Snake variables
    static int snakeLength = 1;
    static int snakeX[MaxSnake], snakeY[MaxSnake];
    static int ptr, nextPtr;
    static int snakeRound = 0;
    static SnakeStates_t SnakeState;
    int attempt;
    boolean continueLoop = true;
  
  if (SensorUpdate.check(MeasurementFreg)) {
    tmpValue  = (mySensor.readTempC()*PrecTemp);
    tmpValue2 = tempTable[dayNumber];
    tempTable[dayNumber] = tmpValue2 + (tmpValue-tmpValue2)/measurementNumber;
//      PRINT("Day: ", dayNumber);
//      PRINT("   Measuremeant: ", measurementNumber);
//      PRINT("  Temp: ", tmpValue);
//      PRINT("  Aver Temp: ", tempTable[dayNumber]);
//      PRINTLN;
    measurementNumber++;
    if (measurementNumber >= MaxMeasurements) {
      measurementNumber = 1;
      dayNumber++;
      if (ClockState==_Clock_Temp) ClockState=_Clock_Temp_init;
      if (dayNumber >= NumberOfPoints ) {
        for (int ii = 0; ii <= NumberOfPoints-2; ii++) {
          tempTable[ii] = tempTable[ii+1];
        }
        tempTable[NumberOfPoints-1] = 0;
        dayNumber = NumberOfPoints-1;
      }
    }
  }
  
  // check the light to setup matrix intensivity after a final write
  if (IntensityCheck.check(IntensityWait)) {
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

  // check if NTP sync is due?
  // If yes change clock status
  if (NTPUpdateTask.check(NTPRESYNC)) {
    ClockState  = _Clock_NTP_Sync;
  }

  // for all status except ... check if the time of DST has came and if yes change the time accordingly
  if ( ClockState != _Clock_init ) {
    correctByDST();
  }

  // check time dependant actions

  // .....   

  // cehck the current clock / display status
  switch(ClockState)
  {
      case _Clock_init:
          ClockState = _Clock_simple_time_init;
          break;
          
      case _Clock_NTP_Sync:
        PRINTT;
        if (StartSyncNTP()) { // error by NTP sync
          zoneInfo0.setText("NTP Sync ERROR", _SCROLL_LEFT, _TO_FULL, InfoTick1, I0s, I0e);
          zoneInfo0.Animate(true);
          SyncNTPError = true;          
        } else {
          zoneInfo0.setText("NTP Sync OK", _SCROLL_LEFT, _TO_FULL, InfoTick1, I0s, I0e);
          zoneInfo0.Animate(true);
          PRINTS("NTP sync OK, Local="); PRINTS(timeClient.getFormattedTime() ); PRINTLN;
          SyncNTPError = false;
        }
        ClockState = goBackState;
        PRINT("NTP Resync Completed\nNew mode=", ClockState); PRINTLN;
        PRINTT;
        break;

      case _Clock_Temp_init:
          clearScreen();
          StatTask.start();
          tempMin = tempTable[0];
          tempMax = tempTable[0];
          for (ptr=0; ptr < dayNumber; ptr++) {
            tmpValue = tempTable[ptr];
          
            PRINT("tmpValue : ", tmpValue);
            PRINT("  min : ", tempMin);
            PRINT("  max : ", tempMax);
            PRINTLN;

            if (tmpValue < tempMin) tempMin = tmpValue;
            if (tmpValue > tempMax) tempMax = tmpValue;
          }
          if (tempMax-tempMin < 8) tempMax = tempMin+8;

          goBackState = _Clock_Temp_init;
          ClockState = _Clock_Temp;
          break;

      case _Clock_Temp:

          if (StatTask.check(20)) {
            clearScreen();            
            for (ptr = 0; ptr <= dayNumber; ptr++) {
              int xtmpValue = map(tempTable[ptr], tempMin, tempMax, 0, 8);
              // xtmpValue = constrain(xtmpValue, 0, 8);
  
//              PRINT("i: ", ptr);
//              PRINT("  Table: ", tempTable[ptr]);
//              PRINT("  Temp : ", xtmpValue);
//              PRINT("  min : ", tempMin);
//              PRINT("  max : ", tempMax);
//              PRINTLN;
  
              matrix.drawFastVLine(ptr, 8-xtmpValue, xtmpValue, HIGH);
            }
            updateDisplay = true;
          }
          key = keyboard(_Clock_complete_info_init, _Clock_simple_time_init, _Clock_none, _Clock_none);
          break;           
                             
      case _Clock_simple_time_init:

          clearScreen();

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
          matrix.drawPixel(H0e+1,2,HIGH);
          matrix.drawPixel(H0e+1,5,HIGH);
          matrix.drawPixel(H0e+1,7, SyncNTPError || !digitalRead(modePin)); // indicate if NTP sync error 
//          PRINT("SyncNTPError =",SyncNTPError);
//          PRINT("   Mode =",digitalRead(modePin));
//          PRINTLN;

          updateDisplay = true;

          //IntensityCheck.start();
          SnakeUpdate.start();
          randomSeed(analogRead(pinRandom)); // Initialize random generator
          SnakeState = _sInit;
          
          PRINTS("Simple Time Init closed\n");
          
          goBackState = ClockState;
          ClockState = _Clock_simple_time;
          break;

      case _Clock_simple_time:
          updateTtime(valueH, lvalueH, hour(), 'H');
          updateTtime(valueM, lvalueM, minute(), 'M');
          if ( updateTtime(valueS, lvalueS, second(), 'S') ) {
//            flasher = !flasher;
//            digitalWrite(ledPin, flasher);
            updateDisplay = true;
            PRINTS(timeClient.getFormattedTime() );
            PRINTLN;
//            matrix.setClip(H0e+1,H0e+2,0,8);
//            matrix.drawPixel(H0e+1,2,flasher);
//            matrix.drawPixel(H0e+1,5,flasher);
          }
          updateDisplay |= zoneClockH0.Animate(false);
          updateDisplay |= zoneClockH1.Animate(false);
          updateDisplay |= zoneClockM0.Animate(false);
          updateDisplay |= zoneClockM1.Animate(false);
          updateDisplay |= zoneClockS1.Animate(false); 
          updateDisplay |= zoneClockS0.Animate(false);

          // Snake animation
          if (SnakeUpdate.check(SnakeWait) || SnakeState ==_sRunA) {
              matrix.setClip(SNs,SNe,0,8);
              updateDisplay = true;
              switch (SnakeState) {
                case _sInit:
//                        PRINTS("\n Init");
                        matrix.fillScreen(LOW);
                        for ( ptr = 0; ptr < snakeLength; ptr++ ) {
                          snakeX[ptr] = SNs+(SNe-SNs) / 2;
                          snakeY[ptr] = matrix.height() / 2;
                        }
                        nextPtr = 0;
                        snakeLength = 1;
                        snakeRound = 0;              
          
                        SnakeState = _sRunA;
                        break;
          
                case _sRunA:              
//                        PRINTS("\n State A");
                        ptr = nextPtr;
                        nextPtr = next(ptr, snakeLength);
                        matrix.drawPixel(snakeX[ptr], snakeY[ptr], HIGH); // Draw the head of the snake
                        SnakeState = _sRunB;
                        break;      
                        
                case _sRunB:
//                        PRINTS("\n State B");
                        if ( !occupied(nextPtr, snakeLength, snakeX, snakeY) ) {
                          matrix.drawPixel(snakeX[nextPtr], snakeY[nextPtr], LOW); // Remove the tail of the snake
                        }
                      
                        continueLoop = true;
                        for ( attempt = 0; (attempt < SnakeAttempt) && continueLoop ; attempt++ ) {
                          // Jump at random one step up, down, left, or right
                          switch ( random(4) ) {
                            case 0: 
                                snakeX[nextPtr] = (snakeX[ptr] + 1 >= SNe) ? SNs : snakeX[ptr] + 1;
                                snakeY[nextPtr] = snakeY[ptr]; 
                                break;
                            case 1:
                                snakeX[nextPtr] = (snakeX[ptr] - 1 < SNs) ? SNe-1 : snakeX[ptr] - 1;      
                                snakeY[nextPtr] = snakeY[ptr]; 
                                break;
                            case 2: 
                                snakeY[nextPtr] = (snakeY[ptr] + 1 > matrix.height()-1)? 0 : snakeY[ptr] + 1;
                                snakeX[nextPtr] = snakeX[ptr]; 
                                break;
                            case 3:
                                snakeY[nextPtr] = (snakeY[ptr] - 1 < 0)? matrix.height()-1 : snakeY[ptr] - 1;
                                snakeX[nextPtr] = snakeX[ptr]; 
                                break;
                          }    
                          continueLoop = occupied(nextPtr, snakeLength, snakeX, snakeY);
                        }
                        if (attempt == SnakeAttempt) { 
                          matrix.fillScreen(HIGH);
                          SnakeState = _sFail;
                        } else {
                            snakeRound = (snakeRound +1) % SankeNextRound;
                            if (snakeRound == 0) snakeLength = snakeLength + 1;
                            if (snakeLength >= MaxSnake) {
                               matrix.fillScreen(HIGH);
                               SnakeState = _sFail;
                            }  else SnakeState = _sRunA;  
                        }
                        break;
                case _sFail:
//                        PRINTS("\n Fail");
                        SnakeState = _sInit;
                        break;
              }
          }

          key = keyboard(_Clock_complete_info_init, _Clock_alarm_init, _Clock_none, _Clock_none);
          break;     

      case _Clock_alarm_init:
          zoneInfo0.setText("Set Up Alarm", _SCROLL_LEFT, _TO_LEFT, 5, I0s, I0e);
          zoneInfo0.Animate(true);
          
          updateDisplay = false;

          // adjustTime(-SECS_PER_HOUR*2);
          // PRINTS("Alarm Init closed\n");
          // Status2Clock_NTP_Sync();
          ClockState = _Clock_Temp_init;
          break;
          
      case _Clock_alarm:
          //PRINTS("Alarm\n");
          updateDisplay = zoneInfo0.Animate(false);
          if (zoneInfo0.AnimateDone()) zoneInfo0.Reset();
          key = keyboard(_Clock_complete_info_init, _Clock_simple_time_init, _Clock_none, _Clock_none);
          break;     
      
      case _Clock_complete_info_init:
       
          clearScreen();
          
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
          matrix.drawPixel(H0e+1,7, SyncNTPError || !digitalRead(modePin)); // indicate if NTP sync error 
//          PRINT("SyncNTPError =",SyncNTPError);
//          PRINT("   Mode =",digitalRead(modePin));
//          PRINTLN;
          
          updateDisplay = true;

          DataMode = 0;
          DataDisplayTask.start(-2000);
          
          //IntensityCheck.start();
          
          PRINTS("Complete Time Init closed\n");

          goBackState = ClockState;
          ClockState = _Clock_complete_info;
          break;

      case _Clock_complete_info:
     
          updateTtime(valueH, lvalueH, hour(), 'H');
          updateTtime(valueM, lvalueM, minute(), 'M');
          if ( updateTtime(valueS, lvalueS, second(), 'S') ) {
            flasher = !flasher;
            // digitalWrite(ledPin, flasher);
            updateDisplay = true;
//            PRINTS(timeClient.getFormattedTime() );
//            PRINTLN;
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
                 sprintf (DataStr, "%s%02d", monthShortStr(month()), day());
                 textEffect = _SCROLL_LEFT;
                 break;
              case 1:
                 sprintf (DataStr, "%s%02d", dayShortStr(weekday()), day());
                 textEffect = _SCROLL_LEFT;
                 break;
              case 99: 
                 sprintf (DataStr, "%s", monthStr(month()));
                 textEffect = _SCROLL_LEFT;
                 break;
              case 2: 
                 // sprintf (DataStr, "%c%d%c", 160, (int)(mySensor.readTempC()+0.5), 161);
                 tmpValue  = tempTable[dayNumber];
                 tmpValue2 = mySensor.readTempC();
                 sprintf (DataStr, "%d%c", (int)(tmpValue2+0.5), 161);                  
                 tmpValue2 *= PrecTemp;
                 
                 // textEffect = tmpValue2 > tmpValue ? _SCROLL_UP : tmpValue2 < tmpValue ? _SCROLL_DOWN:_SCROLL_LEFT;
                 if (tmpValue2 > tmpValue) {
                   textEffect = _SCROLL_UP;                    

                   PRINTS("UP\n");
                   
                 } else if (tmpValue2 < tmpValue) {

                   PRINTS("DOWN\n");
                   
                   textEffect = _SCROLL_DOWN;                                     
                 } else {

                   PRINTS("LEFT\n");
                  
                   textEffect = _SCROLL_LEFT;
                 }
                 

                 
                 
                 break;
              case 3: 
                 //sprintf (DataStr, "%c%.0f%c", 162, mySensor.readFloatPressure()/100, 163);
                 sprintf (DataStr, "%.0f%c", mySensor.readFloatPressure()/100, 163);        
                 textEffect = _SCROLL_LEFT;
                 break;                   
              case 4: 
                 //sprintf (DataStr, "%c%d%%", 166, (int)(mySensor.readFloatHumidity()+0.5));
                 sprintf (DataStr, "%d%%", (int)(mySensor.readFloatHumidity()+0.5));
                 textEffect = _SCROLL_LEFT;
                 break;                   
              case 6: 
                 int measurement = hallRead();
                 PRINT("Hall sensor measurement: ", measurement);
                 sprintf (DataStr, "H:%02d", measurement);
                 textEffect = _SCROLL_LEFT;
                 break;                   
            }
//            PRINTS(DataStr);PRINTLN;
            //zoneInfo1.setText(DataStr, _SCROLL_LEFT, _TO_LEFT, InfoTick1, I1s, I1e);
            zoneInfo1.setText(DataStr, textEffect, _TO_LEFT, InfoTick, I1s, I1e);
            DataMode = (DataMode+1) % 5;
          }
          updateDisplay |= zoneInfo1.Animate(false);
          key = keyboard(_Clock_simple_time_init, _Clock_alarm_init, _Clock_none, _Clock_none);
          break;          
          
      case _Clock_idle:
          break;

      default:;
  }

  if (updateDisplay) {
    matrix.write();
    matrix.setIntensity(intensity);
    updateDisplay = false;
  }

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
        PRINTS("\nDown\n");
      } 
   } else { 
      if (SWPrev == 0) {
        SWPrev = 1;
        x = SW_UP;
        Q.push((uint8_t *)&x);
        PRINTS("\nUp\n");
      } 
   }
}

void processEncoder () 
{
  uint8_t x = R.read();
  if (x)   {
    PRINTS((x == DIR_CW ? "\n+1\n" : "\n-1\n"));
    Q.push(&x);
  }
}



#if DEBUG_ON
void digitalClockDisplay(){
  // digital clock display of the time
  Serial.println();
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
#endif
