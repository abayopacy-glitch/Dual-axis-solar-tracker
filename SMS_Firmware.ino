/*
 * ============================================================
 * DUAL-AXIS SOLAR TRACKER WITH WEATHER & SMS ALERTS
 * Target: Arduino Uno (ATmega328P)
 * Author: Generated Firmware
 * ============================================================
 *
 * PIN ASSIGNMENT SUMMARY
 * ─────────────────────────────────────────────────────────
 * A0  → LDR Top-Left
 * A1  → LDR Top-Right
 * A2  → LDR Bottom-Left
 * A3  → LDR Bottom-Right
 * A4  → I2C SDA  (INA219 + LCD 16×2)
 * A5  → I2C SCL  (INA219 + LCD 16×2)
 * D3  → Rain Sensor (digital out)
 * D4  → DHT11 Data
 * D5  → Servo Pan  (horizontal / azimuth)
 * D6  → Servo Tilt (vertical  / elevation)
 * D10 → SIM800C RX  (Arduino TX → SIM800C)
 * D11 → SIM800C TX  (SIM800C  → Arduino RX)
 *
 * LIBRARIES REQUIRED (install via Library Manager)
 * ─────────────────────────────────────────────────────────
 * Adafruit INA219           (Adafruit)
 * DHT sensor library        (Adafruit)
 * LiquidCrystal I2C         (Frank de Brabander)
 * Servo                     (built-in)
 * SoftwareSerial            (built-in)
 * Wire                      (built-in)
 * ============================================================
 */

// ─── Library includes ───────────────────────────────────────
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// ════════════════════════════════════════════════════════════
//  COMPILE-TIME CONFIGURATION
// ════════════════════════════════════════════════════════════

// — GSM SMS Destination ——————————————————————————————————————
#define ADMIN_PHONE         "+250791452459"

// — Hardware pins ————————————────————────────────────────────
#define LDR_TL        A0
#define LDR_TR        A1
#define LDR_BL        A2
#define LDR_BR        A3
#define RAIN_PIN      3
#define DHT_PIN       4
#define SERVO_PAN_PIN 5
#define SERVO_TLT_PIN 6
#define GSM_TX_PIN    10   // Arduino → SIM800C
#define GSM_RX_PIN    11   // SIM800C → Arduino

// — Servo travel limits (degrees) ————————————————————————────
#define PAN_MIN       0
#define PAN_MAX       180
#define TILT_MIN      10
#define TILT_MAX      90
#define SERVO_INIT_PAN   90
#define SERVO_INIT_TILT  45

// — Tracking thresholds & timing —————————————————————————────
#define THRESHOLD_BASE        50    // base LDR difference threshold
#define THRESHOLD_ADAPT_RATE  0.05f // how fast threshold adapts (EMA factor)
#define SERVO_STEP            1     // degrees per tracking step
#define NIGHT_LDR_LEVEL       80    // 0-1023; below → night
#define TRACK_INTERVAL_MS     500   // sun-tracking cycle (non-blocking)
#define SENSOR_INTERVAL_MS    2000  // DHT/INA read cycle
#define LCD_INTERVAL_MS       3000  // LCD page flip cycle
#define NIGHT_CHECK_MS        5000  // night detection check cycle
#define DECISION_HOLD_MS      800   // debounce: motor only moves if signal persists

// — Protection limits ————————────────————────────────————────
#define OC_THRESHOLD_A        3.0f  // over-current limit (A)
#define OV_THRESHOLD_V        12.0f  // Trigger fault ABOVE 12.0V

// ════════════════════════════════════════════════════════════
//  OBJECT INSTANCES
// ════════════════════════════════════════════════════════════
Adafruit_INA219   ina219;
DHT               dht(DHT_PIN, DHT11);
LiquidCrystal_I2C lcd(0x27, 16, 2);   // adjust I2C address if needed
Servo             servoPan;
Servo             servoTilt;
SoftwareSerial    gsm(GSM_RX_PIN, GSM_TX_PIN);

// ════════════════════════════════════════════════════════════
//  STATE MACHINE
// ════════════════════════════════════════════════════════════
enum SystemState {
  STATE_INIT,
  STATE_TRACKING,
  STATE_NIGHT,
  STATE_RAIN_SAFE,    // tilt panel flat to shed water
  STATE_FAULT
};

SystemState sysState = STATE_INIT;

// ════════════════════════════════════════════════════════════
//  GLOBAL DATA
// ════════════════════════════════════════════════════════════

// — Servo positions ——————————————————————————————————————————
int panAngle  = SERVO_INIT_PAN;
int tiltAngle = SERVO_INIT_TILT;

// — Sensor readings ——————————————————────────────────────────
float busVoltage  = 0;
float current_mA  = 0;
float power_mW    = 0;
float temperature = 0;
float humidity    = 0;
bool  isRaining   = false;
bool  isNight     = false;

// — Adaptive threshold ───────────────────────────────────────
float adaptThreshold = THRESHOLD_BASE;

// — Non-blocking timers ──────────────────────────────────────
unsigned long tTrack      = 0;
unsigned long tSensor     = 0;
unsigned long tLCD        = 0;
unsigned long tNight      = 0;
unsigned long tDecisionH  = 0;   // horizontal debounce timestamp
unsigned long tDecisionV  = 0;   // vertical   debounce timestamp

// — Decision debounce latches ────────────────────────────────
int  pendingPanDir  = 0;   // -1, 0, +1
int  pendingTiltDir = 0;

// — Fault flags ──────────────────────────────────────────────
bool faultOvercurrent   = false;
bool faultUndervoltage  = false;
bool faultOvervoltage   = false;
bool smsSentFault       = false;
bool smsSentRain        = false;

// — LCD page control ─────────────────────────────────────────
uint8_t lcdPage = 0;

// — GSM ready flag ───────────────────────────────────────────
bool gsmReady = false;

// ════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ════════════════════════════════════════════════════════════
void taskTracking();
void taskSensors();
void taskLCD();
void taskNightCheck();
void checkProtections();

void moveServoPan (int deg);
void moveServoTilt(int deg);
void servosNightPark();
void servosRainPark();

bool initGSM();
void sendSMS(const char* msg);
bool waitForResponse(const char* expected, uint16_t timeoutMs);
void gsmSend(const char* cmd);
void gsmFlush();

void updateLCD();
void checkAndClearFaults();
String faultString();

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  gsm.begin(9600);

  // — I2C devices —
  Wire.begin();
  ina219.begin();
  lcd.init();
  lcd.backlight();

  // — DHT —
  dht.begin();

  // — Rain sensor —
  pinMode(RAIN_PIN, INPUT);

  // — Servos —
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TLT_PIN);
  moveServoPan(SERVO_INIT_PAN);
  moveServoTilt(SERVO_INIT_TILT);

  // — LCD splash —
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Solar Tracker ");
  lcd.setCursor(0, 1); lcd.print("  Initialising..");
  delay(2000);

  // — GSM init —
  lcd.clear();
  
  // Live countdown for GSM cell tower registration
  for (int i = 12; i > 0; i--) {
    lcd.setCursor(0, 0); 
    lcd.print("GSM Booting...  ");
    lcd.setCursor(0, 1); 
    lcd.print("Wait: "); 
    if (i < 10) lcd.print(" "); 
    lcd.print(i); 
    lcd.print(" sec   ");
    delay(1000);
  }

  gsmReady = initGSM();

  if (gsmReady) {
    lcd.setCursor(0, 0); lcd.print("GSM: OK         ");
    lcd.setCursor(0, 1); lcd.print("SMS Alerts: ON  ");
  } else {
    lcd.setCursor(0, 0); lcd.print("GSM: NO SIGNAL  ");
    lcd.setCursor(0, 1); lcd.print("Local mode only ");
  }

  delay(2000);
  sysState = STATE_TRACKING;
  lcd.clear();
}  

// ════════════════════════════════════════════════════════════
//  MAIN LOOP  (fully non-blocking)
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Always run sensor & protection checks
  if (now - tSensor >= SENSOR_INTERVAL_MS) {
    tSensor = now;
    taskSensors();
    checkProtections();
  }

  // Night & Rain detection
  if (now - tNight >= NIGHT_CHECK_MS) {
    tNight = now;
    taskNightCheck();
  }

  // LCD update
  if (now - tLCD >= LCD_INTERVAL_MS) {
    tLCD = now;
    updateLCD();
  }

  // State-based tracking
  switch (sysState) {
    case STATE_TRACKING:
      if (now - tTrack >= TRACK_INTERVAL_MS) {
        tTrack = now;
        taskTracking();
      }
      break;

    case STATE_NIGHT:
      // Servos parked; nothing to do until dawn
      break;

    case STATE_RAIN_SAFE:
      // Panel tilted flat; tracking suspended
      break;

    case STATE_FAULT:
      // Await fault clearance
      checkAndClearFaults();
      break;

    default:
      sysState = STATE_TRACKING;
      break;
  }
}

// ════════════════════════════════════════════════════════════
//  TASK: READ SENSORS
// ════════════════════════════════════════════════════════════
void taskSensors() {
  // — INA219 —
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW   = ina219.getPower_mW();

  // — DHT11 —
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperature = t;
  if (!isnan(h)) humidity    = h;

  // — Rain sensor (LOW = rain detected on most modules) —
  isRaining = (digitalRead(RAIN_PIN) == LOW);
}

// ════════════════════════════════════════════════════════════
//  TASK: PROTECTION CHECKS
// ════════════════════════════════════════════════════════════
void checkProtections() {
  float currentA = current_mA / 1000.0f;

  faultOvercurrent  = (currentA   > OC_THRESHOLD_A);
  faultOvervoltage  = (busVoltage  > OV_THRESHOLD_V);
  
  // Hardcoded to false so 0V never triggers an alarm
  faultUndervoltage = false; 

  bool anyFault = faultOvercurrent || faultOvervoltage;

  if (anyFault && sysState != STATE_FAULT) {
    sysState = STATE_FAULT;
    
    // Suspend servos to prevent damage
    servoPan.detach();
    servoTilt.detach();

    if (!smsSentFault && gsmReady) {
      String msg = "SOLAR TRACKER FAULT: " + faultString();
      sendSMS(msg.c_str());
      smsSentFault = true;
    }
  }
}
void checkAndClearFaults() {
  float currentA = current_mA / 1000.0f;
  
  // The system clears the fault if current is safe AND voltage is 6.0V or less
  bool cleared = (currentA   <= OC_THRESHOLD_A) &&
                 (busVoltage <= OV_THRESHOLD_V);
                 
  if (cleared) {
    faultOvercurrent  = false;
    faultUndervoltage = false;
    faultOvervoltage  = false;
    smsSentFault      = false;
    
    // Re-attach servos
    servoPan.attach(SERVO_PAN_PIN);
    servoTilt.attach(SERVO_TLT_PIN);
    sysState = STATE_TRACKING;
  }
}

String faultString() {
  String s = "";
  if (faultOvercurrent)  s += "OVERCURRENT ";
  if (faultUndervoltage) s += "UNDERVOLTAGE ";
  if (faultOvervoltage)  s += "OVERVOLTAGE ";
  return s;
}

// ════════════════════════════════════════════════════════════
//  TASK: NIGHT & RAIN DETECTION
// ════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════
//  TASK: NIGHT & RAIN DETECTION
// ════════════════════════════════════════════════════════════
void taskNightCheck() {
  int tl = analogRead(LDR_TL);
  int tr = analogRead(LDR_TR);
  int bl = analogRead(LDR_BL);
  int br = analogRead(LDR_BR);
  int avg = (tl + tr + bl + br) / 4;

  isNight = (avg < NIGHT_LDR_LEVEL);

  // 1. RAIN OVERRIDE (Highest weather priority)
  if (isRaining) {
    // Park the servos if we aren't already in a fault state
    if (sysState == STATE_TRACKING || sysState == STATE_NIGHT) {
      sysState = STATE_RAIN_SAFE;
      servosRainPark();
    }
    
    // Fire the SMS independently of the day/night state
    if (!smsSentRain && gsmReady) {
      sendSMS("RAIN DETECTED: Panel tilted to safe position.");
      smsSentRain = true; // Lock the rain SMS so it doesn't spam
    }
  } 
  
  // 2. NORMAL WEATHER (Handle Day/Night transitions)
  else {
    // It stopped raining, so unlock the rain SMS for the next storm
    if (smsSentRain) {
      smsSentRain = false; 
    }

    // Only resume sun tracking if there are no active electrical faults
    if (sysState == STATE_RAIN_SAFE || sysState == STATE_TRACKING || sysState == STATE_NIGHT) {
      
      if (isNight) {
        if (sysState != STATE_NIGHT) {
          sysState = STATE_NIGHT;
          servosNightPark();
        }
      } else {
        if (sysState != STATE_TRACKING) {
          sysState = STATE_TRACKING;
          // Servos will automatically resume moving in taskTracking()
        }
      }
      
    }
  }
}
// ════════════════════════════════════════════════════════════
//  TASK: SUN TRACKING (non-blocking, adaptive, debounced)
// ════════════════════════════════════════════════════════════
void taskTracking() {
  // Read all four LDRs
  int tl = analogRead(LDR_TL);
  int tr = analogRead(LDR_TR);
  int bl = analogRead(LDR_BL);
  int br = analogRead(LDR_BR);

  // — Adaptive threshold (EMA) —
  int maxDiff = max(abs(tl - tr), abs(bl - br));
  maxDiff     = max(maxDiff, max(abs(tl - bl), abs(tr - br)));
  adaptThreshold = (1.0f - THRESHOLD_ADAPT_RATE) * adaptThreshold +
                    THRESHOLD_ADAPT_RATE * (float)maxDiff * 0.5f;
  adaptThreshold = constrain(adaptThreshold, 20, 200);

  // — Differential signals —
  int hError = (tl + bl) - (tr + br);   // +ve → sun left  → pan left  (decrease)
  int vError = (tl + tr) - (bl + br);   // +ve → sun above → tilt up   (increase)

  // — Horizontal debounce ——————————————————————————————————
  int newPanDir = 0;
  if      (hError >  (int)adaptThreshold) newPanDir = -1;
  else if (hError < -(int)adaptThreshold) newPanDir = +1;

  if (newPanDir != pendingPanDir) {
    pendingPanDir = newPanDir;
    tDecisionH    = millis();
  } else if (newPanDir != 0 && (millis() - tDecisionH >= DECISION_HOLD_MS)) {
    panAngle = constrain(panAngle + newPanDir * SERVO_STEP, PAN_MIN, PAN_MAX);
    moveServoPan(panAngle);
  }

  // — Vertical debounce ————————————————————────────────────
  int newTiltDir = 0;
  if      (vError >  (int)adaptThreshold) newTiltDir = +1;
  else if (vError < -(int)adaptThreshold) newTiltDir = -1;

  if (newTiltDir != pendingTiltDir) {
    pendingTiltDir = newTiltDir;
    tDecisionV     = millis();
  } else if (newTiltDir != 0 && (millis() - tDecisionV >= DECISION_HOLD_MS)) {
    tiltAngle = constrain(tiltAngle + newTiltDir * SERVO_STEP, TILT_MIN, TILT_MAX);
    moveServoTilt(tiltAngle);
  }
}

// ════════════════════════════════════════════════════════════
//  SERVO HELPERS
// ════════════════════════════════════════════════════════════
void moveServoPan(int deg) {
  deg = constrain(deg, PAN_MIN, PAN_MAX);
  servoPan.write(deg);
}

void moveServoTilt(int deg) {
  deg = constrain(deg, TILT_MIN, TILT_MAX);
  servoTilt.write(deg);
}

void servosNightPark() {
  // Park facing East for sunrise (pan = 90, tilt = minimum)
  moveServoPan(90);
  moveServoTilt(TILT_MIN);
}

void servosRainPark() {
  // Lay panel flat to shed rainwater
  moveServoPan(panAngle);  // keep azimuth
  moveServoTilt(TILT_MIN);
}

// ════════════════════════════════════════════════════════════
//  TASK: LCD UPDATE  (rotating pages)
// ════════════════════════════════════════════════════════════
void updateLCD() {
  lcd.clear();
  char buf[17];

  switch (lcdPage) {
    case 0:
      // Row 0: Temp & Humidity (Using direct lcd.print for floats)
      lcd.setCursor(0, 0);
      lcd.print("T:"); 
      lcd.print(temperature, 1); 
      lcd.print("C H:"); 
      lcd.print(humidity, 0); 
      lcd.print("% ");
      
      // Row 1: Rain status
      lcd.setCursor(0, 1);
      lcd.print(isRaining ? "Rain: YES  SAFE " : "Rain: NO        ");
      break;

    case 1:
      // Row 0 & Row 1: PV Voltage Display Only
      lcd.setCursor(0, 0);
      lcd.print("Panel Voltage:  "); 
      lcd.setCursor(0, 1);
      lcd.print(busVoltage, 2); 
      lcd.print(" V            "); 
      break;
      
      // Row 1: Power
      lcd.setCursor(0, 1);
      lcd.print("Power:"); 
      lcd.print((int)power_mW); 
      lcd.print("mW    ");
      break;

    case 2:
      // Row 0: Servo positions
      snprintf(buf, 17, "Pan:%3d Tilt:%2d", panAngle, tiltAngle);
      lcd.setCursor(0, 0); lcd.print(buf);
      
      // Row 1: State
      lcd.setCursor(0, 1);
      switch (sysState) {
        case STATE_TRACKING:  lcd.print("State: TRACKING "); break;
        case STATE_NIGHT:     lcd.print("State: NIGHT    "); break;
        case STATE_RAIN_SAFE: lcd.print("State: RAIN SAFE"); break;
        case STATE_FAULT:     lcd.print("State: FAULT!   "); break;
        default:              lcd.print("State: INIT     "); break;
      }
      break;

    case 3:
      // Row 0: Fault info
      lcd.setCursor(0, 0); lcd.print("FAULT STATUS:   ");
      lcd.setCursor(0, 1);
      if (!faultOvercurrent && !faultUndervoltage && !faultOvervoltage) {
        lcd.print("All Systems OK  ");
      } else {
        String fs = faultString();
        fs.setCharAt(15, ' '); // Ensure string fits
        lcd.print(fs.substring(0, 16).c_str());
      }
      break;
  }

  lcdPage = (lcdPage + 1) % 4;
}

// ════════════════════════════════════════════════════════════
//  GSM / SIM800C SMS FUNCTIONS
// ════════════════════════════════════════════════════════════

// Flush incoming GSM buffer
void gsmFlush() {
  while (gsm.available()) gsm.read();
}

// Send AT command
void gsmSend(const char* cmd) {
  gsmFlush();
  gsm.println(cmd);
}

// Wait for expected response
bool waitForResponse(const char* expected, uint16_t timeoutMs) {
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (gsm.available()) {
      char c = gsm.read();
      resp += c;
      if (resp.indexOf(expected) >= 0) return true;
    }
  }
  return false;
}

// Initialise SIM800C for SMS only
bool initGSM() {
  // Basic AT handshake
  for (uint8_t i = 0; i < 5; i++) {
    gsmSend("AT");
    if (waitForResponse("OK", 1000)) break;
    if (i == 4) return false;
    delay(500);
  }
  gsmSend("ATE0");           // echo off
  waitForResponse("OK", 500);
  gsmSend("AT+CMGF=1");      // SMS text mode
  waitForResponse("OK", 500);
  return true;
}

// Send SMS
// Send SMS with LCD Feedback
void sendSMS(const char* msg) {
  if (!gsmReady) return;

  // 1. Immediately notify user on the screen
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("ALERT TRIGGERED!");
  lcd.setCursor(0, 1); lcd.print("Sending SMS...  ");

  char cmd[30];
  snprintf(cmd, 30, "AT+CMGS=\"%s\"", ADMIN_PHONE);
  gsmSend(cmd);
  
  // 2. Wait for the '>' prompt
  if (!waitForResponse(">", 3000)) {
    lcd.setCursor(0, 1); lcd.print("SMS Failed!     ");
    delay(2000);
    return; // Exit if the module doesn't respond
  }
  
  // 3. Dispatch the message
  gsm.print(msg);
  gsm.write(26);   // Ctrl+Z to send
  
  // 4. Wait for network confirmation and update LCD
  if (waitForResponse("OK", 8000)) {
    lcd.setCursor(0, 1); lcd.print("SMS Sent OK!    ");
  } else {
    lcd.setCursor(0, 1); lcd.print("SMS Timeout!    ");
  }

  // Hold the status on the screen for 2.5 seconds so you can read it 
  // before the main loop takes over and goes back to standard telemetry
  delay(2500); 
}