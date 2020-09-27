/**
 * Description here
 * blah blah blah
 */

// include the library code
#include <math.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <LiquidCrystal_I2C.h>  // Sunfounder Display

// Setup LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Setup Pressure Sensor
SFE_BMP180 pSensor;

// For Rotary Encoder (KY-040)
int RE_CLOCK_PIN = 3;  // Connected to CLK on KY-040
int RE_DATA_PIN  = 4;  // Connected to DT on KY-040

// Red LED
int RED_LED_PIN = 1;

static uint8_t re_prevNextCode = 0;
uint8_t re_clk_hist = 0;
uint8_t re_clk = 0;
uint8_t re_dta_hist = 0;
uint8_t re_dta = 0;

// Setup the rotary encoder
void setup_rotary()
{
  pinMode(RE_CLOCK_PIN, INPUT);
  pinMode(RE_CLOCK_PIN, INPUT_PULLUP);
  pinMode(RE_DATA_PIN, INPUT);
  pinMode(RE_DATA_PIN, INPUT_PULLUP);
}

uint32_t rotary_ms = 0;
void sample_rotary()
{
  // Sample once every 1 milliseconds
  if (millis() - rotary_ms > 1)
  {  
    rotary_ms = millis();
    re_clk_hist <<= 1;
    re_dta_hist <<= 1;
    if (digitalRead(RE_CLOCK_PIN))
      re_clk_hist |= 0x01;
    if (digitalRead(RE_DATA_PIN))
      re_dta_hist |= 0x01;

    // Two in a row means we set the value
    if (0x07 == (re_clk_hist & 0x07))
      re_clk = 1;
    if (0x00 == (re_clk_hist & 0x07))
      re_clk = 0;
    if (0x07 == (re_dta_hist & 0x07))
      re_dta = 1;
    if (0x00 == (re_dta_hist & 0x07))
      re_dta = 0;
  }
}

// If clk goes high-to-low and data is high, +1
// If clk goes high-to-low and data is low, -1
// Else, 0
uint8_t prev_re_clk = 0;
int8_t get_rotary_value()
{
  //lcd.setCursor(5, 0);
  //lcd.print(re_clk);
  //lcd.setCursor(10, 0);
  //lcd.print(re_dta);
  
  int8_t val = 0;
  if (prev_re_clk > 0 && re_clk < 1)
  { // We went high-to-low
    if (re_dta > 0)
      val = 1;
    else
      val = -1;
  }
  prev_re_clk = re_clk; // save state

  return val;
}

// Gets absolute pressure in millibar, and temp in C
double getAbsPressureMbar(double &tempC)
{
  uint8_t status;
  double pressure = 0;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pSensor.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Function returns 1 if successful, 0 if failure.

    status = pSensor.getTemperature(tempC);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pSensor.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pSensor.getPressure(pressure, tempC);
        if (status != 0)
        {
          return(pressure);
        }
      }
    }
  }
  return -999.99; // error
}

double cToF(double degC)
{
  return degC * 9.0 / 5.0 + 32.0;
}

uint8_t getNumLen(double num)
{
  int len = 1;
  if (num < 0)
    len++;
  num = abs(num);
  if (num >= 10)
    len++;
  if (num >= 100)
    len++;
  if (num >= 1000)
    len++;
  if (num >= 10000)
    len++;
  if (num >= 100000)
    len++;
  return len;
}

void setup()
{
  // Pin for sanity check blink
  //pinMode(red_led, OUTPUT);

  // LCD Init
  lcd.init();
  lcd.noBacklight();
  delay(100);
  lcd.backlight();
  lcd.print("Initializing...");
  delay(500);

  // Rotary Encoder Init
  setup_rotary();

  // Barometric Sensor Init
  if(!pSensor.begin())
  {
    lcd.setCursor(0, 1);
    lcd.print("BMP180 not detected");
    delay(1500);
    lcd.clear();
  }
  else
  {
    lcd.setCursor(0, 1);
    lcd.print("Found BMP180");
    delay(1500);
    lcd.clear();
  }
}

/*********************************************************/
// Globals
uint32_t global_ms = 0; // Track global milliseconds
uint32_t global_secs = 0;
uint32_t last_tenth_ms = 0;
uint32_t last_secs_ms = 0;
double pressureSample = 0;

int prevRotaryVal = -1;
int rotaryVal = 0;

double altFt = 0.0;
double prevAltFt = 0.0;

double degF = 0.0;
double prevDegF = 0.0;

uint8_t last_alt_pos = 0;

uint8_t once = 1;

// loop gets called over and over and over
void loop() 
{
  global_ms = millis();
  sample_rotary();
  rotaryVal += get_rotary_value();
  double inHg = 29.9 + rotaryVal / 100.0;

  // Do this once every second
  if (global_ms - last_secs_ms > 980)
  {
    global_secs++;
    last_secs_ms = global_ms;

    // Reading our temp and pressure
    double tempC = 0.0;
    pressureSample = getAbsPressureMbar(tempC);
    degF = cToF(tempC);

    // We are accurate (I guess...)
    // To .17 meters = 0.5 feet or so. BUT, sitting on the bench, alt in feet seems
    // to drift, and change spuriously by a couple of feet or so.
    if (abs(prevAltFt - altFt) > 4.0)
    {
      prevAltFt = altFt;

      uint8_t pos = getNumLen(altFt);
      pos = 15 - pos - 5; // account for decimals and 'ft'

      // Silly trick to clear out old stuff
      // and only print what's necessary
      if (pos > last_alt_pos)
      {
        lcd.setCursor(0, 0);
        lcd.print("             ");
      }
      last_alt_pos = pos;
      
      lcd.setCursor(pos, 0);
      lcd.print(altFt);
    }

    if (abs(prevDegF - degF) > 1.2)
    {
      lcd.setCursor(0, 1);
      lcd.print(round(degF));
      lcd.print((char) 223);
      lcd.print("F");
    }
  
    // Print num of seconds we've been running
    //lcd.setCursor(0, 0);
    //lcd.print(global_secs);
  }

  // Calculate our values
  {
    // Convert InHG (rotaryVal) to mbar
    double offsetMbar = inHg * 33.86389;
    double altMeters = pSensor.altitude(pressureSample, offsetMbar);
    altFt = altMeters * 3.28084;
  }

  // Always print the change to our rotary value
  if (rotaryVal != prevRotaryVal && global_ms-last_tenth_ms > 100)
  {
    last_tenth_ms = global_ms;
    prevRotaryVal = rotaryVal;
    
    // Print rotary encoder value
    lcd.setCursor(6, 1);
    lcd.print(inHg);
  }

  if (once)
  {
    lcd.setCursor(13, 0);
    lcd.print(" ft");
    lcd.setCursor(12, 1);
    lcd.print("InHg");
    once = 0;
  }
}
/************************************************************/
