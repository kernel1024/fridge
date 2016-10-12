#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <avr/wdt.h>

//#define DEBUG

// ---------- CONSTANTS -----------
#define DEFAULT_SETPOINT 3.0
#define HYSTERESIS 2.0
#define MAXBUF 32
#define SCANTIME_MS 500

#define MIN_VALID_TEMP_READS 5

#ifndef DEBUG

#define MOTOR_MIN_ON 240      // 120*1000/SCANTIME_MS; 2 min
#define MOTOR_MAX_ON 7200     // 3600*1000/SCANTIME_MS; 60 min
#define MOTOR_MIN_OFF 1200    // 600*1000/SCANTIME_MS; 10 min

#else

#define MOTOR_MIN_ON 5*1000/SCANTIME_MS
#define MOTOR_MAX_ON 30*1000/SCANTIME_MS
#define MOTOR_MIN_OFF 10*1000/SCANTIME_MS
#define tempCorrector 25.0

#endif
// --------------------------------

// ------------ PINS --------------
#define LIVE_LED 13

#define BTN_UP 7
#define BTN_DOWN 6

#define DHTPIN 5
#define MOTORPIN 4
// ---------------------------------

// ------------ SENSORS ------------
// DHT. Temperature sensetivity - 0.1 C
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
// ---------------------------------

LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Writes temperature setpoint in flash memory in 3 copies.
// Valid range -10.0 .. +10.0 C
void writeSetpoint(float t) {
  if (t < -10.0) t = -10.0;
  if (t > 10.0) t = 10.0;
  char b = round(t * 10.0);
  EEPROM.write(10, b);
  EEPROM.write(20, b);
  EEPROM.write(30, b);
}

// Reads and validates temperature setpoint from flash memory
float readSetpoint() {
  char s1 = EEPROM.read(10);
  char s2 = EEPROM.read(20);
  char s3 = EEPROM.read(30);
  float res = DEFAULT_SETPOINT;
  if (s1 != s2 || s2 != s3 || s1 != s3) {
    writeSetpoint(res);
  } else {
    res = (s1 / 10.0);
    if (res > 10.0 || res < -10.0) {
      res = DEFAULT_SETPOINT;
      writeSetpoint(res);
    }
    return res;
  }
}

char* char_repeat(char* buf, int n, char c) {
  memset(buf, c, n);
  buf[n] = '\0';
  return buf;
}

float setTemp = DEFAULT_SETPOINT;
float currentTemp = DEFAULT_SETPOINT;
float currentHumid = 0.0;
int rest_count = 0;
int cool_count = 0;
int invTemp_count = 0;
boolean liveLed = false;
boolean motor = false;

void updateLCD() {
  static char s[MAXBUF];
 
  lcd.setCursor(0, 0);
  int tempLen = 12;
  if (isnan(currentTemp)) {
    tempLen -= lcd.print("Temp: -NAN-");
  } else {
    tempLen -= lcd.print("Temp: "); 
    tempLen -= lcd.print(currentTemp, 1);
  }
  if (tempLen>0 && tempLen<MAXBUF) {
    lcd.print(char_repeat(s,tempLen,' '));
  }
  
  lcd.setCursor(12,0);
  if (motor) {
    lcd.print("-ON-");
  } else {
    lcd.print("----");
  }
  
  lcd.setCursor(0, 1);
  tempLen = 12;
  tempLen -= lcd.print("Set:  ");
  tempLen -= lcd.print(setTemp, 1);
  if (tempLen>0 && tempLen<MAXBUF) {
    lcd.print(char_repeat(s,tempLen,' '));
  }

  lcd.setCursor(12,1);
  if (!isnan(currentHumid)) {
    lcd.print(round(currentHumid)); lcd.print("%   ");
  } else {
    lcd.print("    ");
  }
}

void setup() {
  wdt_disable();

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.print("i2c ok");

  invTemp_count = MIN_VALID_TEMP_READS;

  pinMode(BTN_UP, INPUT);
  pinMode(BTN_DOWN, INPUT);

  pinMode(LIVE_LED, OUTPUT);
  pinMode(MOTORPIN, OUTPUT);
  motor = false;
  digitalWrite(MOTORPIN, motor);

  dht.begin();

  rest_count = MOTOR_MIN_OFF;
  cool_count = 0;

  delay(1000);

#ifdef DEBUG
  Serial.begin(9600);
#endif

  wdt_enable(WDTO_8S);
}

boolean tooWarm() {
  return (currentTemp >= (setTemp + HYSTERESIS));
}

boolean restLongEnough() {
  return (rest_count >= MOTOR_MIN_OFF);
}

boolean tooCold() {
  return (currentTemp <= setTemp); //temp keeps dropping after temp is reached
}

boolean onLongEnough() {
  return (cool_count >= MOTOR_MIN_ON);
}

boolean onTooLong() {
  return (cool_count >= MOTOR_MAX_ON);
}

void loop() {
  liveLed = !liveLed;
  digitalWrite(LIVE_LED, liveLed);

  int upBtn = digitalRead(BTN_UP);
  int downBtn = digitalRead(BTN_DOWN);
  setTemp = readSetpoint();
  currentTemp = setTemp;

  if (upBtn > 0) {
    setTemp += 0.1;
    writeSetpoint(setTemp);
  }
  if (downBtn > 0) {
    setTemp -= 0.1;
    writeSetpoint(setTemp);
  }

  float t = dht.readTemperature();
  currentHumid = dht.readHumidity();
  if (isnan(t)) {
    invTemp_count = MIN_VALID_TEMP_READS;
  } else {
    if (invTemp_count == 0) {
#ifndef DEBUG
      currentTemp = t;
#else
      currentTemp = t - tempCorrector;
#endif
    } else {
      invTemp_count--;
    }
  }

  updateLCD();

  wdt_reset();

  if (motor == false) {
    if (tooWarm() && restLongEnough() && (invTemp_count == 0)) {
      //turn fridge on after temp has risen and waiting for compressor to cool down
      motor = true;
      digitalWrite(MOTORPIN, motor);
      delay(1000);
      cool_count = 0;
    }
    rest_count++;
  } else { //fridge status is COOLING

    if (onTooLong() || (onLongEnough() && tooCold()))
    {
      //turn the fridge off if its cool enough or compressor on too long
      motor = false;
      digitalWrite(MOTORPIN, motor);
      delay(1000);
      rest_count = 0;
    }
    cool_count++;
  }

  delay(SCANTIME_MS);

  wdt_reset();
}
