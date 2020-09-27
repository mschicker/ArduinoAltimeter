/**
 * Description here
 * blah blah blah
 */

// include the library code
#include <Wire.h>               // Need for I2C
#include <LiquidCrystal_I2C.h>  // Sunfounder Display

// Setup LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// For Rotary Encoder (KY-040)
int pinA = 3;  // Connected to CLK on KY-040
int pinB = 4;  // Connected to DT on KY-040
int encoderPosCount = 0; 
int pinALast;  
unsigned short aVal = 0;
boolean bCW;

// Red LED
int red_led = 1;

// Globals
int secs = 0;
int cycle = 0;

/*
void blink (int n)
{
  for (int i = 0; i < n; i++)
  {
    digitalWrite(red_led, HIGH); 
    delay(75);
    digitalWrite(red_led, LOW);
    delay(75);
  }
  delay(1000);
}
*/

void readEncoder()
{
  delay(1);
  aVal = aVal << 1; // shift one bit
  aVal = aVal | digitalRead(pinA);

  lcd.setCursor(8, 1);
  lcd.print(aVal);

  if ((0x0000 == aVal & 0x00FF) && (0 != pinALast))
  {
    pinALast = 0;
    if (digitalRead(pinB))
    {
      encoderPosCount++;
    }
    else
    {
      encoderPosCount--;
    }
  }
  else if (0x00FF == aVal & 0x00FF)
  {
    pinALast = 1;
  }
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
  delay(1000);
  lcd.clear();

  // Encoder Init
  pinMode (pinA,INPUT);
  pinMode (pinB,INPUT);
  aVal = 0;
  pinALast = 0;
}

/*********************************************************/
void loop() 
{
  int one_sec = 101;
  lcd.setCursor(8, 0);
  lcd.print(one_sec);
  
  // Second count
  if (cycle == one_sec)
  {
    lcd.setCursor(0, 0);
    lcd.print(secs);
    secs++;
    cycle = 0;
  }

  // Do Encoder
  if (true || 0 == cycle % 10)
  {
    readEncoder();
    lcd.setCursor(0, 1);
    lcd.print(encoderPosCount);
  }
  
  // Increment our cycle
  cycle++;
}
/************************************************************/
