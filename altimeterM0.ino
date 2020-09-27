#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// OLED display TWI address
#define OLED_ADDR   0x3C

// Check OLED display setup...
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Declare display
Adafruit_SSD1306 display(-1);

// Declare Pressure Sensor
// Note: had to change address to 0x76 in SFE_BMP180.h
Adafruit_BMP280 pSensor; // I2C pSensor;

void setupBMP280()
{
  /* Default settings from datasheet. */
  pSensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

// For Rotary Encoder (KY-040)
int RE_CLOCK_PIN = 4;  // Connected to CLK on KY-040
int RE_DATA_PIN  = 3;  // Connected to DT on KY-040

// Red LED
int RED_LED_PIN = 13;

// Rotary
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

uint32_t rotary_ms = 0; // global rotary timing var
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
// Note that millibar is the same as hPa (hectopascal)
double getAbsPressureMbar(double &tempC)
{
  tempC = pSensor.readTemperature();
  double pressure = pSensor.readPressure();

  return(pressure);
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
  // initialize red led
  pinMode(RED_LED_PIN, OUTPUT);
  
  // initialize and clear display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  // size 1, 10px high, 21 chars wide
  // size 2, 20px high, 10 chars wide
  // Yellow is 15px high
  // Then blue
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Initializing...");

  // Rotary Encoder Init
  setup_rotary();

  // Barometric Sensor Init
  if(!pSensor.begin())
  {
    display.setCursor(0,16);
    display.print("BMP280 not detected");
  }
  else
  {
    display.setCursor(0,16);
    display.print("Found BMP280");

    setupBMP280();
  }

  // update display with all of the above graphics
  display.display();
  delay(2000);
}

/*********************************************************/
// Globals
uint32_t global_ms = 0; // Track global milliseconds
uint32_t global_secs = 0;
uint32_t last_tenth_ms = 0;
uint32_t last_sec_ms = 0;
double pressureSample = 0;

int prevRotaryVal = -1;
int rotaryVal = 0;

double altFt = 0.0;

double degF = 0.0;
double prevDegF = 0.0;

uint8_t last_alt_pos = 0;

uint8_t once = 1;

int count = 0;
void loop()
{
  global_ms = millis();
  sample_rotary();
  rotaryVal += get_rotary_value();
  double inHg = 29.9 + rotaryVal / 100.0;
  
  bool updateDisplay = false;

  // Update rotary display 1/10 second since our last update
  if (rotaryVal != prevRotaryVal && global_ms-last_tenth_ms > 100)
  {
    prevRotaryVal = rotaryVal;
    last_tenth_ms = global_ms;
    last_sec_ms = global_ms;
    updateDisplay = true;
  }
  
  if (rotaryVal == prevRotaryVal && global_ms - last_sec_ms > 1000)
  {
    last_sec_ms = global_ms;
    updateDisplay = true;

    // Reading our temp and pressure
    double tempC = 0.0;
    pressureSample = getAbsPressureMbar(tempC);
    degF = cToF(tempC);

    {
      // Convert InHG (rotaryVal) to mbar
      double offsetMbar = inHg * 33.86389;
      double altMeters = pSensor.readAltitude(offsetMbar);
      double tempAltFt = altMeters * 3.28084;

      // Alt can jiggle around a bit, so we apply a dead band of 1.5 ft.
      if (abs(altFt - tempAltFt) > 1.5)
      {
        altFt = tempAltFt;
      }
    }
  }

  
  // Only update display if we change rotary value, or once per second
  if (updateDisplay)
  {
    display.clearDisplay();
    //display.fillRect(0, 0, 64, 20, 0);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print (inHg);
    display.print (" inHg");

    
    display.setCursor(90, 0);
    display.print(round(degF));
    display.print((char) 247);
    display.print("F");
    
    display.setCursor(0,30);
    display.setTextSize(2);
    display.print(round(altFt));
    display.print ("ft");
    
    display.display();
  }
  
}

