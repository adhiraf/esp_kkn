#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pins for ultrasonic sensor
const int trigPin = 25;
const int echoPin = 26;

// Relay pin
const int ledPin = 5;

// Create MPU6050 instance
Adafruit_MPU6050 mpu;

// Create LCD instance with I2C address 0x27 and 20x4 display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Timer variables
unsigned long motionDetectedTime = 0;
bool ledActive = false;

void setup(void) {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // Initialize LCD
  lcd.init();        // Correct method to initialize the LCD
  lcd.backlight();   // Turn on the backlight

  // Setup pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  delay(100);
}

void loop() {
  if (mpu.getMotionInterruptStatus()) {
    // Motion detected
    motionDetectedTime = millis(); // Update the last motion detected time
    ledActive = true;

    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print out the values
    Serial.println("Detect Motion!");
  }

  // Activate relay for 1 minute if motion was detected
  if (ledActive) {
    if (millis() - motionDetectedTime < 5000) { // 5 seconds
      digitalWrite(ledPin, HIGH);
    } else {
      ledActive = false;
      digitalWrite(ledPin, LOW);
    }
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Measure distance using ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  delay(1000);
  
  // Row 1: "Tinggi Muka Air" centered
  lcd.setCursor((20 - strlen("Tinggi Muka Air")) / 2, 0);
  lcd.print("Tinggi Muka Air");

  // Row 2: Distance with " cm" centered
  String distanceStr = String(distance) + " cm";
  lcd.setCursor((20 - distanceStr.length()) / 2, 1);
  lcd.print(distanceStr);

  // Row 3 : blank strip
  lcd.setCursor(0,2);
  lcd.print("--------------------");

  // Row 4: "KKN STMKG Unit 15" centered
  lcd.setCursor((20 - strlen("KKN STMKG Unit 15")) / 2, 3);
  lcd.print("KKN STMKG Unit 15");

  delay(500);  // Delay to avoid flickering
}
