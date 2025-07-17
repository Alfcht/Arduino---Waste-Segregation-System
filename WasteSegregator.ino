#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Servo Motors
Servo paperServo;
Servo plasticServo;

// Pin Definitions
const int trigPin = 9;
const int echoPin = 10;
const int irPin = 7;
const int colorPin = A0;  // Potentiometer as color sensor substitute

// RGB LED Pins
const int redPin = 3;
const int greenPin = 5;
const int bluePin = 6;

// Servo Pins
const int paperServoPin = 11;
const int plasticServoPin = 12;

// Variables
long duration;
int distance;
bool objectDetected = false;
int colorValue;
String materialType = "";

void setup() {
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Garbage Sorter");
  lcd.setCursor(0, 1);
  lcd.print("Ready...");
  
  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  // Attach servos
  paperServo.attach(paperServoPin);
  plasticServo.attach(plasticServoPin);
  
  // Set servos to initial position (closed)
  paperServo.write(0);
  plasticServo.write(0);
  
  // Test RGB LED colors on startup
  Serial.println("Testing RGB LED...");
  setRGBColor(255, 0, 0); // Red
  delay(1000);
  setRGBColor(0, 255, 0); // Green
  delay(1000);
  setRGBColor(0, 0, 255); // Blue
  delay(1000);
  setRGBColor(255, 255, 255); // White (ready state)
  
  delay(2000);
  lcd.clear();
}

void loop() {
  // Check for object presence using ultrasonic sensor
  distance = getDistance();
  
  // Read color sensor (potentiometer) continuously
  colorValue = analogRead(colorPin);
  
  // Print sensor values to Serial Monitor for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("cm, Color Value: ");
  Serial.println(colorValue);
  
  // Display current sensor readings on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print(distance);
  lcd.print("cm Val:");
  lcd.print(colorValue);
  
  // Display material type based on potentiometer value
  lcd.setCursor(0, 1);
  if (colorValue < 300) {
    lcd.print("Type: PAPER");
  } else if (colorValue > 700) {
    lcd.print("Type: PLASTIC");
  } else {
    lcd.print("Type: UNKNOWN");
  }
  
  // Check IR sensor for object confirmation
  int irValue = digitalRead(irPin);
  
  // Object detected when distance < 15cm (increased threshold)
  if (distance < 15) {
    if (!objectDetected) {
      objectDetected = true;
      
      Serial.println("OBJECT DETECTED!");
      
      // Clear display and show detection message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("GARBAGE DETECTED");
      lcd.setCursor(0, 1);
      lcd.print("Analyzing...");
      
      // Set LED to yellow (analyzing)
      setRGBColor(255, 255, 0);
      Serial.println("LED set to YELLOW (analyzing)");
      
      delay(1500);
      
      // Read color sensor value again for sorting
      colorValue = analogRead(colorPin);
      Serial.print("Final color value for sorting: ");
      Serial.println(colorValue);
      
      // Determine material type and sort
      if (colorValue < 300) {
        materialType = "PAPER";
        sortPaper();
      } else if (colorValue > 700) {
        materialType = "PLASTIC";
        sortPlastic();
      } else {
        materialType = "UNKNOWN";
        handleUnknown();
      }
      
      delay(3000); // Wait for sorting to complete
      
      // Return servos to initial position
      Serial.println("Returning servos to closed position");
      paperServo.write(0);
      plasticServo.write(0);
      
      delay(2000); // Wait before ready for next object
    }
  } else {
    objectDetected = false;
    // Set LED to white (ready)
    setRGBColor(255, 255, 255);
  }
  
  delay(500); // Slower refresh rate for better LCD readability
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  return distance;
}

void sortPaper() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PAPER DETECTED!");
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  lcd.print(colorValue);
  lcd.print(" Sorting");
  
  // Set LED to GREEN for paper (FIXED)
  setRGBColor(0, 255, 0);
  Serial.println("LED set to GREEN for PAPER");
  
  // Open paper compartment - servo on pin 11 rotates to 90 degrees
  Serial.println("Opening paper compartment - servo pin 11 to 90 degrees");
  paperServo.write(90);
  
  Serial.println("Paper detected and sorted - Value: " + String(colorValue));
}

void sortPlastic() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PLASTIC DETECTED");
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  lcd.print(colorValue);
  lcd.print(" Sorting");
  
  // Set LED to RED for plastic (FIXED)
  setRGBColor(255, 0, 0);
  Serial.println("LED set to RED for PLASTIC");
  
  // Open plastic compartment - servo on pin 12 rotates to 90 degrees
  Serial.println("Opening plastic compartment - servo pin 12 to 90 degrees");
  plasticServo.write(90);
  
  Serial.println("Plastic detected and sorted - Value: " + String(colorValue));
}

void handleUnknown() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UNKNOWN MATERIAL");
  lcd.setCursor(0, 1);
  lcd.print("Value: ");
  lcd.print(colorValue);
  lcd.print(" Manual!");
  
  // Set LED to orange for unknown (changed from red since red is now for plastic)
  setRGBColor(255, 165, 0);
  
  Serial.println("Unknown material detected - Value: " + String(colorValue));
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}