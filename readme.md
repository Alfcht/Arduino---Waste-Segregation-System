My profile - (default - not available to a different person who logs on. not available if I log on to a different computer.)
C:\Users\userName\Documents\Arduino\libraries

IDE installation folder - (libraries added here will be removed with next IDE upgrade.)
C:\Program Files (x86)\Arduino\libraries

Libraries folder inside your sketchbook - (not available to other sketches.)
C:\Users\userName\Documents\Arduino\sketchName\libraries


#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// TCS34725 Color Sensor Setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// Servo Motors
Servo leftServo;   // Paper bin servo
Servo rightServo;  // Plastic bin servo

// Pin Definitions
const int trigPin = 9;
const int echoPin = 10;
const int irPin = 7;

// Individual LED Pins
const int redLED = 3;
const int greenLED = 4;
const int blueLED = 5;

// Servo Pins
const int leftServoPin = 11;   // Paper bin servo (left side)
const int rightServoPin = 12;  // Plastic bin servo (right side)

// Variables
long duration;
int distance;
bool objectDetected = false;
bool objectProcessed = false;
uint16_t r, g, b, c;
String materialType = "";

// Timing variables
unsigned long lastDetectionTime = 0;
const unsigned long detectionCooldown = 3000; // 3 seconds cooldown

void setup() {
  Serial.begin(9600);
  
  // Initialize Wire library for I2C communication
  Wire.begin();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Waste Sorter");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  // Initialize TCS34725 color sensor
  if (tcs.begin()) {
    Serial.println("TCS34725 color sensor found");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Color Sensor");
    lcd.setCursor(0, 1);
    lcd.print("ERROR!");
    while (1);
  }
  
  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irPin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  
  // Attach servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  
  // Set servos to initial position (horizontal/closed)
  leftServo.write(0);   // Horizontal position
  rightServo.write(0);  // Horizontal position
  
  // Test LEDs on startup
  Serial.println("Testing LEDs...");
  
  // Turn on red LED
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  delay(1000);
  
  // Turn on green LED
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, LOW);
  delay(1000);
  
  // Turn on blue LED
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, HIGH);
  delay(1000);
  
  // Turn on white (all LEDs)
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  delay(1000);
  
  // Turn off all LEDs (ready state)
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");
  
  Serial.println("System initialized successfully!");
}

void loop() {
  // Check for object presence using ultrasonic sensor (sideways detection)
  bool ultrasonicDetection = detectObjectUltrasonic();
  
  // Check IR sensor for object confirmation (sideways detection)
  bool irDetection = digitalRead(irPin) == LOW; // Assuming LOW when object blocks IR
  
  // Object detected when either sensor triggers
  bool currentObjectDetected = ultrasonicDetection || irDetection;
  
  // Display current status on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("US:");
  lcd.print(ultrasonicDetection ? "YES" : "NO");
  lcd.print(" IR:");
  lcd.print(irDetection ? "YES" : "NO");
  
  lcd.setCursor(0, 1);
  if (currentObjectDetected) {
    lcd.print("Object Detected!");
  } else {
    lcd.print("Waiting...");
  }
  
  // Process object detection with cooldown
  if (currentObjectDetected && !objectDetected && 
      (millis() - lastDetectionTime > detectionCooldown)) {
    
    objectDetected = true;
    lastDetectionTime = millis();
    
    Serial.println("OBJECT DETECTED IN CHUTE!");
    
    // Clear display and show detection message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WASTE DETECTED");
    lcd.setCursor(0, 1);
    lcd.print("Analyzing...");
    
    // Set LED to blue (analyzing)
    setLEDColor("blue");
    Serial.println("LED set to BLUE (analyzing)");
    
    delay(1000); // Give time for object to settle
    
    // Read color sensor values for sorting
    readColorSensor();
    Serial.print("Color values - R:");
    Serial.print(r);
    Serial.print(" G:");
    Serial.print(g);
    Serial.print(" B:");
    Serial.print(b);
    Serial.print(" C:");
    Serial.println(c);
    
    // Determine material type and sort
    materialType = classifyMaterial();
    
    if (materialType == "PAPER") {
      sortPaper();
    } else if (materialType == "PLASTIC") {
      sortPlastic();
    } else {
      handleUnknown();
    }
    
    delay(2000); // Wait for sorting to complete
    
    // Return servos to initial position (horizontal)
    Serial.println("Returning servos to horizontal position");
    leftServo.write(0);
    rightServo.write(0);
    
    // Turn off LEDs
    setLEDColor("off");
    
    delay(1000); // Wait before ready for next object
    
    objectDetected = false;
    
  } else if (!currentObjectDetected) {
    objectDetected = false;
  }
  
  delay(100); // Fast refresh rate for better detection
}

bool detectObjectUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  // Object detected when distance is within detection range (adjust as needed)
  // For sideways detection, this threshold may need adjustment
  return (distance > 2 && distance < 20); // Detect objects 2-20cm from sensor
}

void readColorSensor() {
  tcs.getRawData(&r, &g, &b, &c);
  
  // Optional: Add a small delay to ensure stable reading
  delay(100);
}

String classifyMaterial() {
  // Classification based on TCS34725 color characteristics
  // These thresholds may need adjustment based on your specific materials
  
  // Calculate color ratios for better classification
  float redRatio = (float)r / (float)c;
  float greenRatio = (float)g / (float)c;
  float blueRatio = (float)b / (float)c;
  
  Serial.print("Color ratios - R:");
  Serial.print(redRatio);
  Serial.print(" G:");
  Serial.print(greenRatio);
  Serial.print(" B:");
  Serial.println(blueRatio);
  
  // Paper typically appears white/light colored (balanced RGB values)
  if (c > 1000 && redRatio > 0.3 && greenRatio > 0.3 && blueRatio > 0.3) {
    return "PAPER";
  }
  // Plastic can have various colors but typically different characteristics
  else if (c > 500 && (redRatio > 0.4 || greenRatio > 0.4 || blueRatio > 0.4)) {
    return "PLASTIC";
  }
  else {
    return "UNKNOWN";
  }
}

void sortPaper() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PAPER DETECTED!");
  lcd.setCursor(0, 1);
  lcd.print("Sorting LEFT");
  
  // Set LED to GREEN for paper
  setLEDColor("green");
  Serial.println("LED set to GREEN for PAPER");
  
  // Open left servo (paper bin) - rotate to vertical position
  Serial.println("Opening left servo (paper bin) to vertical position");
  leftServo.write(90); // Vertical position to open chute
  
  Serial.print("Paper detected and sorted - Color values: R=");
  Serial.print(r);
  Serial.print(" G=");
  Serial.print(g);
  Serial.print(" B=");
  Serial.print(b);
  Serial.print(" C=");
  Serial.println(c);
}

void sortPlastic() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PLASTIC DETECTED");
  lcd.setCursor(0, 1);
  lcd.print("Sorting RIGHT");
  
  // Set LED to RED for plastic
  setLEDColor("red");
  Serial.println("LED set to RED for PLASTIC");
  
  // Open right servo (plastic bin) - rotate to vertical position
  Serial.println("Opening right servo (plastic bin) to vertical position");
  rightServo.write(90); // Vertical position to open chute
  
  Serial.print("Plastic detected and sorted - Color values: R=");
  Serial.print(r);
  Serial.print(" G=");
  Serial.print(g);
  Serial.print(" B=");
  Serial.print(b);
  Serial.print(" C=");
  Serial.println(c);
}

void handleUnknown() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UNKNOWN MATERIAL");
  lcd.setCursor(0, 1);
  lcd.print("Manual Sort!");
  
  // Set LED to white for unknown (all LEDs on)
  setLEDColor("white");
  
  Serial.print("Unknown material detected - Color values: R=");
  Serial.print(r);
  Serial.print(" G=");
  Serial.print(g);
  Serial.print(" B=");
  Serial.print(b);
  Serial.print(" C=");
  Serial.println(c);
}

void setLEDColor(String color) {
  // Turn off all LEDs first
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  
  // Turn on specific LED(s) based on color
  if (color == "red") {
    digitalWrite(redLED, HIGH);
  }
  else if (color == "green") {
    digitalWrite(greenLED, HIGH);
  }
  else if (color == "blue") {
    digitalWrite(blueLED, HIGH);
  }
  else if (color == "white") {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
    digitalWrite(blueLED, HIGH);
  }
  // "off" or any other value keeps all LEDs off
}