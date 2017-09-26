/*
 * Arduino Pro Mini with 128x32 OLED Display Module with I2C interface header
 * Also display a 4-bar battery icon on the top right hand corner
 * I2C Option: BME280 Temperature, Humidity, Pressure Sensor
 * I2C Option: INA219 High Side Current Sensor (26V,±3.2A) 
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Compiler directives, comment out to disable
#define USE_SERIAL Serial           // Valid options: Serial and Serial1
//#define I2C_BME280_ADDR 0x76      // BME280 Temperature/Humidity/Pressure Sensor
//#define I2C_INA219_ADDR 0x40      // INA219 High Side Current Sensor 

// To read a max 4.2V from V(bat), a voltage divider 330KΩ-100KΩ-GND is used to drop max voltage 
// down to 1.08V (=Vref)
const float volt_div_const = 4.30*1.08/1.023;           // multiplier = Vin_max*Vref/1.023 (mV)

// SOC table for 75%, 50%, 25% and <5%  (mV)
const int bat_soc_table[] = { 3970, 3860, 3750, 3600 }; // eBay_501235_180mAh @20mA
const int bat_hys_theshold = 30;     // (mV) Hysteresis threshold. Only implemented on FULL/EMPTY 

const int   vbat_pin = A6;        // ADC Pin connected to VBAT
const int   led_flip_mode = 1;    // 1 = LED display is rotated 180
const float led_altitude = 30;    // My altitude (meters)

const unsigned int display_refresh_rate = 1 * 1000; // Display refresh interval = 1 sec
const unsigned int mode_cycling_rate    = 3 * 1000; // Meter mode cycling interval = 3 sec

enum Battery { EMPTY_BAR, ONE_BAR, TWO_BAR,   // Full = 100%,  Three-bar = 75%,  Two-bar = 50%
               THREE_BAR, FULL_BAR };         // One-bar = 25%,  Empty = <5%
Battery batCurrStatus, batPrevStatus;

enum MeterMode { DISP_TEMP, DISP_HUMID };     // Display cycle between temperature and humidity
MeterMode meterMode;
unsigned long modeStartTime;

#ifdef USE_SERIAL
String readBuffer ="";                        // Console input, global variable so value persist
#endif

// use online image converter => http://www.online-utility.org/image_converter.jsp?outputType=XBM
int batWidth=13;
int batHeight=7;
static const unsigned char bat_full_bitmap[] U8X8_PROGMEM = {
  0xFC, 0x1F, 0x07, 0x10, 0x55, 0x15, 0x55, 0x15, 0x55, 0x15, 0x07, 0x10, 0xFC, 0x1F };
static const unsigned char bat_threebar_bitmap[] U8X8_PROGMEM = {
  0xFC, 0x1F, 0x07, 0x10, 0x45, 0x15, 0x45, 0x15, 0x45, 0x15, 0x07, 0x10, 0xFC, 0x1F };
static const unsigned char bat_twobar_bitmap[] U8X8_PROGMEM = {
  0xFC, 0x1F, 0x07, 0x10, 0x05, 0x15, 0x05, 0x15, 0x05, 0x15, 0x07, 0x10, 0xFC, 0x1F };
static const unsigned char bat_onebar_bitmap[] U8X8_PROGMEM = {
  0xFC, 0x1F, 0x07, 0x10, 0x05, 0x14, 0x05, 0x14, 0x05, 0x14, 0x07, 0x10, 0xFC, 0x1F };
static const unsigned char bat_empty_bitmap[] U8X8_PROGMEM = {
  0xFC, 0x1F, 0x07, 0x10, 0x05, 0x10, 0x05, 0x10, 0x05, 0x10, 0x07, 0x10, 0xFC, 0x1F };

// Initialize class objects
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   
#ifdef I2C_BME280_ADDR
Adafruit_BME280 bme; 
#endif
#ifdef I2C_INA219_ADDR
Adafruit_INA219 ina219(I2C_INA219_ADDR);
#endif


void blinkLED()
{
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED on (HIGH is the voltage level)
    delay(1000);                    // wait for a second
    digitalWrite(LED_BUILTIN, LOW);     // turn the LED off by making the voltage LOW
    delay(1000);                    // wait for a second
  }
}


int readBatteryVoltage() 
{
  int volt = analogRead(vbat_pin) * volt_div_const;
  // Take average of two readings to get rid of noise
  delay(1);
  volt = ( volt + analogRead(vbat_pin)*volt_div_const ) / 2 ;
  return volt;
}


// Initialize the battery icon bitmap function
void setupBatteryIcon(int mV) 
{
  if ( mV > bat_soc_table[0] )
    batCurrStatus = batPrevStatus = FULL_BAR;
  else if ( mV > bat_soc_table[1] )
    batCurrStatus = batPrevStatus = THREE_BAR;
  else if ( mV > bat_soc_table[2] )
    batCurrStatus = batPrevStatus = TWO_BAR;
  else if ( mV > bat_soc_table[3] )
    batCurrStatus = batPrevStatus = ONE_BAR;
  else
    batCurrStatus = batPrevStatus = EMPTY_BAR;
}


// Returns the battery icon bitmap for the corresponding voltage input
// To avoid flickering between two states, we make sure the state machine
// never goes back to the previous state.
const unsigned char * getBatteryIconBitmap(int mV) 
{
  switch (batCurrStatus) {
  case FULL_BAR:
    if ((mV < bat_soc_table[0]) && (batPrevStatus != THREE_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = THREE_BAR;
      return bat_threebar_bitmap;
    }
    if (mV > bat_soc_table[0]+bat_hys_theshold) 
      batPrevStatus = batCurrStatus;
    return bat_full_bitmap;
  case THREE_BAR:
    if ((mV > bat_soc_table[0]) && (batPrevStatus != FULL_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = FULL_BAR;
      return bat_full_bitmap;
    }
    if ((mV < bat_soc_table[1]) && (batPrevStatus != TWO_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = TWO_BAR;
      return bat_twobar_bitmap;
    }
    return bat_threebar_bitmap;
  case TWO_BAR:
    if ((mV > bat_soc_table[1]) && (batPrevStatus != THREE_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = THREE_BAR;
      return bat_threebar_bitmap;
    }
    if ((mV < bat_soc_table[2]) && (batPrevStatus != ONE_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = ONE_BAR;
      return bat_onebar_bitmap;
    }
    return bat_twobar_bitmap;
  case ONE_BAR:
    if ((mV > bat_soc_table[2]) && (batPrevStatus != TWO_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = TWO_BAR;
      return bat_twobar_bitmap;
    }
    if ((mV < bat_soc_table[3]) && (batPrevStatus != EMPTY_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = EMPTY_BAR;
      return bat_empty_bitmap;
    }
    return bat_onebar_bitmap;
  case EMPTY_BAR:
    if ((mV > bat_soc_table[3]) && (batPrevStatus != ONE_BAR)) {
      batPrevStatus = batCurrStatus;    
      batCurrStatus = ONE_BAR;
      return bat_onebar_bitmap;
    }
    if (mV < bat_soc_table[3]-bat_hys_theshold) 
      batPrevStatus = batCurrStatus;
    return bat_empty_bitmap;
  } // end of switch()
  
} // end of getBatteryIconBitmap()


void setup(void) 
{
  #ifdef USE_SERIAL
  USE_SERIAL.begin(57600);    // 8MHz ATmega328 can't go as fast as 115200, drop down a notch.
  delay(10);
  USE_SERIAL.println();
  USE_SERIAL.println();
  #endif

  // Beginning hardware checks
  #ifdef I2C_BME280_ADDR
  if (!bme.begin(I2C_BME280_ADDR)) {
    #ifdef USE_SERIAL
    USE_SERIAL.println(F("Error: Couldn't find a BME280 sensor."));
    #endif
    blinkLED();
  }
  delay(10);   // wait for BME280 to initiallize
  meterMode = DISP_TEMP;  // Initiallize diplay mode
  modeStartTime = millis();
  #endif //I2C_BME280_ADDR
  #ifdef I2C_INA219_ADDR
  ina219.begin();
  #endif //I2C_INA219_ADDR

  // Initiallize U8g2 library
  u8g2.begin();
  u8g2.setFlipMode(led_flip_mode);
  
  // Set Vref=1.1V, this makes ADC working correctly even when Vbat goes below 3.3V  
  analogReference(INTERNAL);  
  setupBatteryIcon(readBatteryVoltage());
  
} // end of setup()


void displayVoltage(float volt) 
{
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 8, "Voltage");
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.setCursor(20, 32);
  u8g2.print(volt/1000.0F,3);
  u8g2.setFont(u8g2_font_helvR10_tr);
  u8g2.drawStr(108, 32, "V");
}


void displayTemperature(float temp) 
{
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 8, "Temperature");
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.setCursor(30, 32);
  u8g2.print(temp,1);
  u8g2.setFont(u8g2_font_helvR10_tf);
  char s[] = {0xb0,0x43,0x00};
  u8g2.drawStr(108, 32, s);
}


void displayHumidity(float rh) 
{
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 8, "Humidity");
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.setCursor(30, 32);
  u8g2.print(rh,1);
  u8g2.setFont(u8g2_font_helvR10_tf);
  char s[] = "% ";
  u8g2.drawStr(108, 32, s);
}


void displayCurrent(float amp) 
{
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 8, "Current");
  u8g2.setFont(u8g2_font_helvB18_tr);
  if (abs(amp) < 1000) {
    u8g2.setCursor(30, 32);
    u8g2.print(amp,0);
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(108, 32, "mA");
  } else {
    u8g2.setCursor(20, 32);
    u8g2.print(amp/1000.0F,3);
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(108, 32, "A");
  }
}


void loop(void) 
{
  int batteryVoltage = readBatteryVoltage();
  #ifdef USE_SERIAL
  USE_SERIAL.println("Battery: "+String(batteryVoltage)+"mV  ");
  #endif
  #ifdef I2C_BME280_ADDR
  // Measure BME280 sensors
  float bmeTemperature = bme.readTemperature();
  float bmeHumidity = bme.readHumidity();
  float seaLevelPressure = bme.seaLevelForAltitude(led_altitude,bme.readPressure()) / 100.0F; //(hPa)
  if ( millis()-modeStartTime >= mode_cycling_rate ) {
    if (meterMode == DISP_TEMP) 
      meterMode = DISP_HUMID;
    else if (meterMode == DISP_HUMID) 
      meterMode = DISP_TEMP;
    modeStartTime = millis();
  }
  #ifdef USE_SERIAL
  USE_SERIAL.println("BME280: "+String(bmeTemperature,1)+"C, " + String(bmeHumidity,1) + "%, " + String(seaLevelPressure,1) + "hPa"); 
  #endif
  #endif //I2C_BME280_ADDR
  #ifdef I2C_INA219_ADDR
  float inaCurrent = ina219.getCurrent_mA();
  #ifdef USE_SERIAL
  USE_SERIAL.println("INA219: "+String(inaCurrent,0)+"mA"); 
  #endif
  #endif //I2C_INA219_ADDR
  #ifdef USE_SERIAL
  // Allow developer to test value by enter it through console
  if (USE_SERIAL.available()) {
    readBuffer="";
    while (USE_SERIAL.available()) {
      // Read every byte until <newline> encountered
      // Note: make sure the terminal sends <newline> for every end of line
      char c = USE_SERIAL.read();
      if (c != '\n')
        readBuffer += c;
    }
  }
  if (readBuffer.length() >0) {
    int newval = readBuffer.toInt();
    USE_SERIAL.println("New Value: \""+readBuffer+"\" ");
    // Change the variable to the reading you are testing
    batteryVoltage = newval;  
  }
  #endif //USE_SERIAL
  const unsigned char * bitmap = getBatteryIconBitmap(batteryVoltage);
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(115,0, batWidth, batHeight, bitmap);
    #if !defined(I2C_BME280_ADDR) && !defined(I2C_INA219_ADDR)
    displayVoltage(batteryVoltage);
    #endif // No I2C_ directive defined 
    #ifdef I2C_BME280_ADDR
    if (meterMode == DISP_TEMP) 
      displayTemperature(bmeTemperature);
    else if (meterMode == DISP_HUMID)
      displayHumidity(bmeHumidity);
    #endif //I2C_BME280_ADDR
    #ifdef I2C_INA219_ADDR
    displayCurrent(inaCurrent);
    #endif //I2C_INA219_ADDR
  } while ( u8g2.nextPage() );
  delay(display_refresh_rate);
} // end of loop()

