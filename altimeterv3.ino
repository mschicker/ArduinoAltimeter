/**
 * Description here
 * blah blah blah
 */

// include the library code
#include <TinyWireM.h>
#include <TinyLiquidCrystal.h>  // Sunfounder Display

// Setup LCD
TinyLiquidCrystal lcd(0x27); // set the LCD address to 0x27

// Setup Pressure Sensor
//SFE_BMP180 pSensor;

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

/*
double getPressure()
{
  uint8_t status;
  double temp = 0;
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
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pSensor.getTemperature(temp);
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
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pSensor.getPressure(pressure,temp);
        if (status != 0)
        {
          return(pressure);
        }
      }
    }
  }
  return -999.99; // error
}
*/

void setup()
{
  lcd.begin(16, 2); //16 chars, 2 line display

  lcd.setBacklight(1); // turn backlight on
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

  // Rotary Encoder Init
  setup_rotary();

/*
  // Barometric Sensor Init
  if(!pSensor.begin())
  {
    lcd.setCursor(0, 0);
    lcd.print("BMP180 not detected");
    delay(1000);
    lcd.clear();
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("Found BMP180");
    delay(1000);
    lcd.clear();
  }
  */
}

/*********************************************************/
// Globals
uint32_t global_ms = 0; // Track global milliseconds
uint32_t global_secs = 0;
uint32_t last_print_ms = 0;
int16_t prevRotaryVal = 0;
int16_t rotaryVal = 0;

// loop gets called over and over and over
void loop() 
{
  global_ms = millis();
  sample_rotary();
  rotaryVal += get_rotary_value();
    
  // Do this once every second
  if (global_ms - last_print_ms > 900)
  {
    global_secs++;
    last_print_ms = global_ms;
  
    // Print num of seconds we've been running
    lcd.setCursor(0, 0);
    lcd.print(global_secs);
  }

  if (rotaryVal != prevRotaryVal)
  {
    prevRotaryVal = rotaryVal;
    
    // Print rotary encoder value
    lcd.setCursor(0, 1);
    lcd.print(rotaryVal);
    lcd.print("     ");
  }

  //double pressure = getPressure();
  //lcd.setCursor(6, 0);
  //lcd.print(pressure);
}
/************************************************************/
