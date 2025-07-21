#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// TCS34725 Color Sensor Setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_240MS, TCS34725_GAIN_1X);

// Servo Motors
Servo leftServo;
Servo rightServo;

// Pin Definitions
const int trigPin = 9;
const int echoPin = 10;
const int irPin = 7;
const int redLED = 3;
const int greenLED = 4;
const int blueLED = 5;
const int leftServoPin = 11;
const int rightServoPin = 12;

// Variables - optimized for memory
long duration;
int distance;
bool objectDetected = false;
uint16_t r, g, b, c;
bool colorSensorAvailable = false;
unsigned long lastDetectionTime = 0;
const unsigned long detectionCooldown = 3000;

// Color analysis variables - reduced memory footprint
const byte numReadings = 5;  // Reduced from 7 to save memory
uint16_t ambientC = 0;
bool ambientCalibrated = false;

// Calibration data storage
struct MaterialProfile {
  float rRatio;
  float gRatio;
  float bRatio;
  uint16_t avgClear;
  bool calibrated;
};

MaterialProfile paperProfile = {0, 0, 0, 0, false};
MaterialProfile plasticProfile = {0, 0, 0, 0, false};
bool systemCalibrated = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print(F("Waste Sorter"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));
  
  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irPin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  
  setLEDs(0, 0, 0);
  delay(1000);
  
  // Initialize color sensor
  lcd.clear();
  lcd.print(F("Testing Sensor"));
  colorSensorAvailable = initTCS();
  
  if (!colorSensorAvailable) {
    lcd.clear();
    lcd.print(F("SENSOR FAILED"));
    lcd.setCursor(0, 1);
    lcd.print(F("Check Wiring"));
    while(1) {
      setLEDs(1, 0, 1);
      delay(500);
      setLEDs(0, 0, 0);
      delay(500);
    }
  }
  
  // Initialize servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  leftServo.write(0);
  rightServo.write(0);
  
  // Test LEDs
  lcd.clear();
  lcd.print(F("Testing LEDs"));
  setLEDs(1, 0, 0); delay(300);
  setLEDs(0, 1, 0); delay(300);
  setLEDs(0, 0, 1); delay(300);
  setLEDs(0, 0, 0);
  
  lcd.clear();
  lcd.print(F("System Ready"));
  delay(1000);
  
  // Perform initial calibration
  performInitialCalibration();
}

// Perform initial material calibration
void performInitialCalibration() {
  lcd.clear();
  lcd.print(F("CALIBRATION"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));
  delay(2000);
  
  // Calibrate Paper
  lcd.clear();
  lcd.print(F("PAPER Sample"));
  lcd.setCursor(0, 1);
  lcd.print(F("Place & Wait"));
  setLEDs(0, 1, 1); // Cyan for paper calibration
  
  delay(5000); // Give user time to place paper sample
  
  uint32_t paperR = 0, paperG = 0, paperB = 0, paperC = 0;
  byte paperSamples = 0;
  
  for (byte i = 0; i < 3; i++) {
    lcd.clear();
    lcd.print(F("PAPER Run "));
    lcd.print(i + 1);
    lcd.setCursor(0, 1);
    lcd.print(F("Scanning..."));
    
    readColor();
    if (c > 50) { // Valid reading
      paperR += r;
      paperG += g;
      paperB += b;
      paperC += c;
      paperSamples++;
      
      setLEDs(0, 1, 0); // Green flash for successful reading
      delay(500);
      setLEDs(0, 1, 1);
    }
    delay(1000);
  }
  
  if (paperSamples > 0) {
    paperProfile.avgClear = paperC / paperSamples;
    paperProfile.rRatio = (float)(paperR / paperSamples) / paperProfile.avgClear;
    paperProfile.gRatio = (float)(paperG / paperSamples) / paperProfile.avgClear;
    paperProfile.bRatio = (float)(paperB / paperSamples) / paperProfile.avgClear;
    paperProfile.calibrated = true;
    
    Serial.print(F("Paper Profile - R:"));
    Serial.print(paperProfile.rRatio, 3);
    Serial.print(F(" G:"));
    Serial.print(paperProfile.gRatio, 3);
    Serial.print(F(" B:"));
    Serial.print(paperProfile.bRatio, 3);
    Serial.print(F(" C:"));
    Serial.println(paperProfile.avgClear);
  }
  
  // Wait before plastic calibration
  lcd.clear();
  lcd.print(F("Remove Paper"));
  lcd.setCursor(0, 1);
  lcd.print(F("Wait 3 sec..."));
  setLEDs(0, 0, 0);
  delay(3000);
  
  // Calibrate Plastic
  lcd.clear();
  lcd.print(F("PLASTIC Sample"));
  lcd.setCursor(0, 1);
  lcd.print(F("Place & Wait"));
  setLEDs(1, 0, 1); // Magenta for plastic calibration
  
  delay(5000); // Give user time to place plastic sample
  
  uint32_t plasticR = 0, plasticG = 0, plasticB = 0, plasticC = 0;
  byte plasticSamples = 0;
  
  for (byte i = 0; i < 3; i++) {
    lcd.clear();
    lcd.print(F("PLASTIC Run "));
    lcd.print(i + 1);
    lcd.setCursor(0, 1);
    lcd.print(F("Scanning..."));
    
    readColor();
    if (c > 50) { // Valid reading
      plasticR += r;
      plasticG += g;
      plasticB += b;
      plasticC += c;
      plasticSamples++;
      
      setLEDs(1, 0, 0); // Red flash for successful reading
      delay(500);
      setLEDs(1, 0, 1);
    }
    delay(1000);
  }
  
  if (plasticSamples > 0) {
    plasticProfile.avgClear = plasticC / plasticSamples;
    plasticProfile.rRatio = (float)(plasticR / plasticSamples) / plasticProfile.avgClear;
    plasticProfile.gRatio = (float)(plasticG / plasticSamples) / plasticProfile.avgClear;
    plasticProfile.bRatio = (float)(plasticB / plasticSamples) / plasticProfile.avgClear;
    plasticProfile.calibrated = true;
    
    Serial.print(F("Plastic Profile - R:"));
    Serial.print(plasticProfile.rRatio, 3);
    Serial.print(F(" G:"));
    Serial.print(plasticProfile.gRatio, 3);
    Serial.print(F(" B:"));
    Serial.print(plasticProfile.bRatio, 3);
    Serial.print(F(" C:"));
    Serial.println(plasticProfile.avgClear);
  }
  
  systemCalibrated = paperProfile.calibrated && plasticProfile.calibrated;
  
  lcd.clear();
  if (systemCalibrated) {
    lcd.print(F("CALIBRATION"));
    lcd.setCursor(0, 1);
    lcd.print(F("COMPLETE!"));
    setLEDs(0, 1, 0); // Green
  } else {
    lcd.print(F("CALIBRATION"));
    lcd.setCursor(0, 1);
    lcd.print(F("FAILED!"));
    setLEDs(1, 0, 0); // Red
  }
  
  delay(3000);
  setLEDs(0, 0, 0);
  
  lcd.clear();
  lcd.print(F("Remove Sample"));
  lcd.setCursor(0, 1);
  lcd.print(F("Ready to Sort"));
  delay(2000);
}

// Enhanced TCS initialization with optimal settings
bool initTCS() {
  for (byte i = 0; i < 3; i++) {
    lcd.setCursor(0, 1);
    lcd.print(F("Attempt "));
    lcd.print(i + 1);
    
    if (tcs.begin()) {
      delay(300);
      
      // Test reading
      uint16_t tr, tg, tb, tc;
      tcs.getRawData(&tr, &tg, &tb, &tc);
      if (tc > 50) {
        lcd.setCursor(0, 1);
        lcd.print(F("SUCCESS!"));
        delay(500);
        calibrateAmbient();
        return true;
      }
    }
    delay(1000);
  }
  return false;
}

// Quick ambient light calibration
void calibrateAmbient() {
  lcd.clear();
  lcd.print(F("Calibrating..."));
  
  uint32_t sum = 0;
  for (byte i = 0; i < 5; i++) {
    uint16_t tr, tg, tb, tc;
    tcs.getRawData(&tr, &tg, &tb, &tc);
    sum += tc;
    delay(100);
  }
  
  ambientC = sum / 5;
  ambientCalibrated = true;
  
  lcd.setCursor(0, 1);
  lcd.print(F("Done"));
  delay(1000);
}

void loop() {
  if (!colorSensorAvailable) return;
  
  bool usDetection = checkUltrasonic();
  bool irDetection = digitalRead(irPin) == LOW;
  bool detected = usDetection || irDetection;
  
  if (!objectDetected) {
    lcd.clear();
    lcd.print(F("US:"));
    lcd.print(usDetection ? F("Y") : F("N"));
    lcd.print(F(" IR:"));
    lcd.print(irDetection ? F("Y") : F("N"));
    lcd.setCursor(0, 1);
    lcd.print(F("Waiting..."));
  }
  
  if (detected && !objectDetected && 
      (millis() - lastDetectionTime > detectionCooldown)) {
    processObject();
  } else if (!detected) {
    objectDetected = false;
  }
  
  delay(100);
}

// Enhanced color reading with multiple samples
void readColor() {
  if (!colorSensorAvailable) return;
  
  uint32_t rSum = 0, gSum = 0, bSum = 0, cSum = 0;
  byte validReadings = 0;
  
  for (byte i = 0; i < numReadings; i++) {
    uint16_t tempR, tempG, tempB, tempC;
    delay(60);
    tcs.getRawData(&tempR, &tempG, &tempB, &tempC);
    
    if (tempC > 50) {
      rSum += tempR;
      gSum += tempG;
      bSum += tempB;
      cSum += tempC;
      validReadings++;
    }
  }
  
  if (validReadings > 0) {
    r = rSum / validReadings;
    g = gSum / validReadings;
    b = bSum / validReadings;
    c = cSum / validReadings;
  }
}

// Enhanced classification using calibrated profiles
byte classify() {
  if (c < 50) return 0; // Too dark to classify
  
  if (!systemCalibrated) {
    // Fallback to original algorithm if calibration failed
    return classifyFallback();
  }
  
  float rRatio = (float)r / c;
  float gRatio = (float)g / c;
  float bRatio = (float)b / c;
  
  // Calculate similarity to each calibrated profile
  float paperDistance = sqrt(
    pow(rRatio - paperProfile.rRatio, 2) +
    pow(gRatio - paperProfile.gRatio, 2) +
    pow(bRatio - paperProfile.bRatio, 2)
  );
  
  float plasticDistance = sqrt(
    pow(rRatio - plasticProfile.rRatio, 2) +
    pow(gRatio - plasticProfile.gRatio, 2) +
    pow(bRatio - plasticProfile.bRatio, 2)
  );
  
  // Debug output
  Serial.print(F("R:")); Serial.print(r);
  Serial.print(F(" G:")); Serial.print(g);
  Serial.print(F(" B:")); Serial.print(b);
  Serial.print(F(" C:")); Serial.print(c);
  Serial.print(F(" PaperDist:")); Serial.print(paperDistance, 3);
  Serial.print(F(" PlasticDist:")); Serial.print(plasticDistance, 3);
  
  // Classification based on closest match with confidence threshold
  const float maxDistance = 0.15; // Maximum acceptable distance for classification
  
  if (paperDistance < plasticDistance && paperDistance < maxDistance) {
    Serial.println(F(" -> PAPER"));
    return 1;
  } else if (plasticDistance < paperDistance && plasticDistance < maxDistance) {
    Serial.println(F(" -> PLASTIC"));
    return 2;
  }
  
  Serial.println(F(" -> UNKNOWN"));
  return 0;
}

// Fallback classification method (original algorithm)
byte classifyFallback() {
  float rRatio = (float)r / c;
  float gRatio = (float)g / c;
  float bRatio = (float)b / c;
  
  float maxRatio = max(max(rRatio, gRatio), bRatio);
  float minRatio = min(min(rRatio, gRatio), bRatio);
  float colorBalance = maxRatio - minRatio;
  float rgbSum = rRatio + gRatio + bRatio;
  
  Serial.print(F(" Bal:")); Serial.print(colorBalance, 2);
  Serial.print(F(" Sum:")); Serial.print(rgbSum, 2);
  
  // Original classification logic
  if (c >= 80 && c <= 150) {
    if (rRatio < 0.38 && colorBalance < 0.12 && rgbSum > 0.65) {
      Serial.println(F(" -> PAPER"));
      return 1;
    }
    if (bRatio > gRatio && bRatio > rRatio && bRatio < 0.45) {
      Serial.println(F(" -> PAPER"));
      return 1;
    }
  }
  
  if (c >= 60 && c <= 200 && colorBalance < 0.15 && 
      rRatio < 0.40 && rgbSum > 0.60) {
    Serial.println(F(" -> PAPER"));
    return 1;
  }
  
  if (c >= 100 && c <= 180) {
    if (rRatio > 0.35 && rRatio > gRatio && rRatio > bRatio) {
      Serial.println(F(" -> PLASTIC"));
      return 2;
    }
    if (colorBalance > 0.08 && rgbSum > 0.65) {
      Serial.println(F(" -> PLASTIC"));
      return 2;
    }
  }
  
  if (c >= 80 && c <= 250) {
    // Strong red dominance
    if (rRatio > 0.38 && (rRatio - gRatio) > 0.05) {
      Serial.println(F(" -> PLASTIC"));
      return 2;
    }
    
    // High intensity with color imbalance
    if (c > 120 && colorBalance > 0.10) {
      Serial.println(F(" -> PLASTIC"));
      return 2;
    }
  }
  
  Serial.println(F(" -> UNKNOWN"));
  return 0;
}

// Enhanced material detection with stability check
void processObject() {
  objectDetected = true;
  lastDetectionTime = millis();
  
  lcd.clear();
  lcd.print(F("ANALYZING..."));
  setLEDs(0, 0, 1);
  delay(1000);
  
  // Take multiple readings for stability
  byte results[3];
  for (byte i = 0; i < 3; i++) {
    readColor();
    results[i] = classify();
    delay(300);
  }
  
  // Majority vote
  byte material;
  if (results[0] == results[1] || results[0] == results[2]) {
    material = results[0];
  } else if (results[1] == results[2]) {
    material = results[1];
  } else {
    material = 0;
  }
  
  Serial.print(F("Final: "));
  Serial.println(material);
  
  if (material == 1) {
    lcd.clear();
    lcd.print(F("PAPER"));
    lcd.setCursor(0, 1);
    lcd.print(F("LEFT BIN"));
    setLEDs(0, 1, 0);
    leftServo.write(90);
  } 
  else if (material == 2) {
    lcd.clear();
    lcd.print(F("PLASTIC"));
    lcd.setCursor(0, 1);
    lcd.print(F("RIGHT BIN"));
    setLEDs(1, 0, 0);
    rightServo.write(90);
  } 
  else {
    lcd.clear();
    lcd.print(F("UNKNOWN"));
    lcd.setCursor(0, 1);
    lcd.print(F("NO SORT"));
    setLEDs(1, 1, 1);
  }
  
  delay(2500);
  
  leftServo.write(0);
  rightServo.write(0);
  setLEDs(0, 0, 0);
  objectDetected = false;
  delay(3000);
}

bool checkUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return false;
  
  distance = duration * 0.034 / 2;
  return (distance > 2 && distance < 25);
}

void setLEDs(bool red, bool green, bool blue) {
  digitalWrite(redLED, red);
  digitalWrite(greenLED, green);
  digitalWrite(blueLED, blue);
}