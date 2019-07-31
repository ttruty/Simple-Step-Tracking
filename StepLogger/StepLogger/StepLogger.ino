// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "RTClib.h"
// include the SD library:
#include <SPI.h>
#include <SD.h>
// include the bluefruitlib
#include <bluefruit.h>

#define force  A1
#define PIN_VBAT A7;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
BLEDis bledis;
BLEUart bleuart;
BLEBas  blebas;  // battery


MPU6050 accelgyro(0x69); // <-- use for AD0 high
RTC_PCF8523 rtc; //real time clock object

//Battery 
uint32_t vbat_pin = PIN_VBAT;   

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

#ifdef NRF52840_XXAA    // if this is for nrf52840
#define VBAT_DIVIDER      (0.5F)               // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
#else
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#endif
 
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)



// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
//On ESP8266, the SD CS pin is on GPIO 15
//On Atmel M0, M4, 328p or 32u4 it's on GPIO 10
//On Teensy 3.x it's on GPIO 10
//On STM32F2/WICED, its on PB5
//On ESP32, it's on GPIO 33
//On nRF52832, it's on GPIO 11
//On nRF52840, it's on GPIO 10
const int chipSelect = 11;

int16_t ax, ay, az;
int16_t gx, gy, gz;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

float readVBAT(void) {
  float raw;
 
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);
 
  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14
 
  // Let the ADC settle
  delay(1);
 
  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);
 
  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
 
  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}
 
uint8_t mvToPercent(float mvolts) {
  if(mvolts<3300)
    return 0;
 
  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }
 
  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}



void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);
 
    while ( !Serial ) delay(10);   // for nrf52840 with native usb

    if (! rtc.begin()) 
    {
      Serial.println("Couldn't find RTC");
      while (1);
    }

    if (! rtc.initialized()) 
    {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Bluefruit.autoConnLed(true);

    float batt = readVBAT(); // battery
    uint8_t battPer = mvToPercent(batt);
    Serial.print("Battery = ");
    Serial.println(String(battPer));
    

    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
    Bluefruit.setName("SimpleStepTrack");
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
    bledis.begin();
    // Start BLE Battery Service
    blebas.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();
  
    // Set up and start advertising
    startAdv();

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);


    Serial.print("\nInitializing SD card...");

     // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) 
    {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
    Serial.println("card initialized.");
}

void loop() {

    float batt = readVBAT(); // battery
    uint8_t battPer = mvToPercent(batt);
    //Serial.print("Battery = ");
    //Serial.println(String(battPer));
    blebas.write(battPer);
    
    String dataString = "";

//    if (Bluefruit.connected() && bleuart.notifyEnabled())
//    {
//        float batt = readVBAT(); // battery
//        blebas.write(batt);//    bleuart.print(delimit);
//        bleuart.print("Battery = ");
//        bleuart.println(batt);
//
//    }
    DateTime now = rtc.now();
//  while (Bluefruit.connected() && bleuart.notifyEnabled())
//  {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int forceRead = analogRead(force);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    dataString += String(now.unixtime());
    dataString += String(",");
    dataString += String(ax);
    dataString += String(",");
    dataString += String(ay);
    dataString += String(",");
    dataString += String(az);
    dataString += String(",");
    dataString += String(gx);
    dataString += String(",");
    dataString += String(gy);
    dataString += String(",");
    dataString += String(gz);
    dataString += String(",");
    dataString += String(forceRead);
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print(now.unixtime());Serial.print("\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.print(gz); Serial.print("\t");
        Serial.println(String(forceRead));
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
       // delay(2);
        bleuart.write((uint8_t)(ax >> 8)); bleuart.write((uint8_t)(ax & 0xFF));
        bleuart.write((uint8_t)(ay >> 8)); bleuart.write((uint8_t)(ay & 0xFF));
        bleuart.write((uint8_t)(az >> 8)); bleuart.write((uint8_t)(az & 0xFF));
        bleuart.write((uint8_t)(gx >> 8)); bleuart.write((uint8_t)(gx & 0xFF));
        bleuart.write((uint8_t)(gy >> 8)); bleuart.write((uint8_t)(gy & 0xFF));
        bleuart.write((uint8_t)(gz >> 8)); bleuart.write((uint8_t)(gz & 0xFF));
    #endif

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    
      
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);    
    uint16_t forceInt  = forceRead;
    //String accString = String(xRead) + ',' + String(yRead) + ',' + String(zRead);
    char delimit = ',';
    //bleuart.println(forceInt);
//    bleuart.print(delimit);
//    bleuart.print(ay);
//    bleuart.print(delimit);
//    bleuart.print(az);
//    bleuart.print(delimit);
//    bleuart.print(gx);
//    bleuart.print(delimit);
//    bleuart.print(gy);
//    bleuart.print(delimit);
//    bleuart.print(gz);
//    bleuart.println();
    
//    bleuart.print(x);
//    bleuart.print(delimit);
//    bleuart.print(y);
//    bleuart.print(delimit);
//    bleuart.println(z);
//     }
}

void startAdv(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // There is no room for Name in Advertising packet
  // Use Scan response for Name
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void connect_callback(uint16_t conn_handle)
{
  (void) conn_handle;
  Serial.println("Connected");
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("Disconnected");
}

/**************************************************************************/
/*!
    @brief  Get user input from Serial
*/
/**************************************************************************/
char* getUserInput(void)
{
  static char inputs[64+1];
  memset(inputs, 0, sizeof(inputs));

  // wait until data is available
  while( Serial.available() == 0 ) { delay(1); }

  uint8_t count=0;
  do
  {
    count += Serial.readBytes(inputs+count, 64);
  } while( (count < 64) && Serial.available() );

  return inputs;
}
