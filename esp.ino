#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <Wire.h>
#define MQTT_MAX_PACKET_SIZE 512

// Replace with your network credentials
const char* ssid = "Kecamatan 02";
const char* password = "sabarsabar1"; // replace with your actual password

// Telegram BOT Token and Chat ID
#define BOTtoken "7279363754:AAEHcLfiYkuF5MmRPTubiNm_5rTXZOAEc-c" // Replace with your BOT token
#define CHAT_ID "-4201858654"   // Replace with your chat ID

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Initialize MQTT
const char* mqtt_server = "broker.emqx.io";
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Telegram bot request delay
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

// Pins for ultrasonic sensor
const int trigPin = 25;
const int echoPin = 26;

// LED pin
const int ledPin = 5;

// Create MPU6050 instance
Adafruit_MPU6050 mpu;

// Create LCD instance with I2C address 0x27 and 20x4 display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Constants
const int riverbedHeight = 250; // Example riverbed height in cm

// Timer variables
unsigned long motionDetectedTime = 0;
bool ledActive = false;
unsigned long lastPublishTime = 0; // Last time MQTT message was published
const unsigned long publishInterval = 5000; // 5 seconds
unsigned long lastTelegramTime = 0; // Last time Telegram message was sent
const unsigned long telegramInterval = 60000; // 1 minutes

// Variables to hold TMA and hazard level
int lastTMA = 0;
int lastPumpStatus = 0;
int lastHazardLevel = 1;

// Variables for ultrasonic sensor filtering
const int numReadings = 10; // Using 10 measurements for moving average
long readings[numReadings];
int readIndex = 0;
long total = 0;
long average = 0;

// Monitoring state
bool monitoringActive = false;  // Variable to track monitoring state

// Function prototypes
void setup_wifi();
bool reconnect();
void sendMQTTData(int tma, int pumpStatus, int hazardLevel);
long getUltrasonicReading();
long correctReading(long distance);
void initSensorReadings();
void updateLCD(int TMA, int hazardLevel);
void handleNewMessages(int numNewMessages);
void sendTelegramData();

// Handle new Telegram messages
void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    String text = bot.messages[i].text;
    Serial.println(text);

    if (text == "/status") {
      sendTelegramData();
    } else if (text == "/monitor") {
      monitoringActive = !monitoringActive;
      bot.sendMessage(CHAT_ID, monitoringActive ? "Monitoring aktif" : "Monitoring non-aktif", "");
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Setup pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize sensor readings
  initSensorReadings();

  // Connect to WiFi
  setup_wifi();

  // Setup MQTT
  mqttClient.setServer(mqtt_server, 1883);

  // Setup Telegram
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  bot.sendMessage(CHAT_ID, "Bot Started", "");
}

void loop() {
  unsigned long currentTime = millis();

  // Handle motion detection and LED activation
  if (mpu.getMotionInterruptStatus()) {
    motionDetectedTime = currentTime;
    ledActive = true;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.println("Detect Motion!");
  }

  if (ledActive && (currentTime - motionDetectedTime < 5000)) {
    digitalWrite(ledPin, HIGH);
  } else {
    ledActive = false;
    digitalWrite(ledPin, LOW);
  }

  int pumpStatus = ledActive ? 1 : 0;
  lastPumpStatus = pumpStatus;

  // Measure distance using ultrasonic sensor with moving average
  long averageDistance = getUltrasonicReading();

  int TMA = riverbedHeight - averageDistance;
  lastTMA = TMA;

  Serial.print("Average distance: ");
  Serial.print(averageDistance);
  Serial.print(" cm, TMA: ");
  Serial.print(TMA);
  Serial.println(" cm");

  int hazardLevel;
  String hazardLevelStr;
  if (TMA >= 150) {
    hazardLevel = 3;
    hazardLevelStr = "BAHAYA";
  } else if (TMA >= 100) {
    hazardLevel = 2;
    hazardLevelStr = "WASPADA";
  } else {
    hazardLevel = 1;
    hazardLevelStr = "AMAN";
  }
  lastHazardLevel = hazardLevel;

  updateLCD(lastTMA, lastHazardLevel);

  if (currentTime - lastPublishTime >= publishInterval) {
    sendMQTTData(lastTMA, pumpStatus, lastHazardLevel);
    lastPublishTime = currentTime;
  }

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Check if monitoring is active and handle continuous sending to Telegram
  if (monitoringActive && (lastHazardLevel == 2 || lastHazardLevel == 3) && (currentTime - lastTelegramTime >= telegramInterval)) {
    sendTelegramData();
    lastTelegramTime = currentTime;
  }

  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

  delay(1000);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

bool reconnect() {
  Serial.print("Attempting MQTT connection...");
  // Create a unique client ID
  String clientId = "ESP32Client_" + String(random(0xffff), HEX);
  // Attempt to connect
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    mqttClient.publish("KKNSTMKG15/status", "Connected");
    // ... and resubscribe
    // mqttClient.subscribe("inTopic"); // Uncomment if you have subscriptions
    return true;
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 3 seconds");
    delay(3000);
    return false;
  }
}

void sendMQTTData(int tma, int pumpStatus, int hazardLevel) {
  StaticJsonDocument<100> doc;

  doc["TMA"] = tma;
  doc["PumpStatus"] = pumpStatus;
  doc["HazardLevel"] = hazardLevel;

  char jsonBuffer[100];
  serializeJson(doc, jsonBuffer);

  Serial.println("Publishing data!");
  if (!mqttClient.publish("KKNSTMKG15/parameter", jsonBuffer)) {
    Serial.println("Failed to publish message");
  } else {
    Serial.println("Publish Completed");
  }
}

long getUltrasonicReading() {
  long sum = 0;
  for (int i = 0; i < numReadings; i++) {
    long duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    sum += distance;

    delay(50);
  }

  long averageDistance = sum / numReadings;
  long correctedDistance = correctReading(averageDistance);

  return correctedDistance;
}

long correctReading(long distance) {
  long correctedDistance = distance;

  if (distance >= 70 && distance <= 80) {
    correctedDistance -= 15;
  } else if (distance > 80 && distance <= 90) {
    correctedDistance -= 10;
  } else if (distance > 90 && distance <= 100) {
    correctedDistance -= 5;
  }
  return correctedDistance;
}

void initSensorReadings() {
  for (int i = 0; i < numReadings; i++) {
    readings[i] = getUltrasonicReading();
    total += readings[i];
  }
  average = total / numReadings;
}

void updateLCD(int TMA, int hazardLevel) {
  lcd.clear();
  delay(10);

  // Row 1: "Tinggi Muka Air" centered
  lcd.setCursor((20 - strlen("Tinggi Muka Air")) / 2, 0);
  lcd.print("Tinggi Muka Air");

  // Row 2: Distance with " cm" centered
  String distanceStr = String(TMA) + " cm";
  lcd.setCursor((20 - distanceStr.length()) / 2, 1);
  lcd.print(distanceStr);

  // Row 3: Hazard level centered
  String hazardLevelStr;
  switch (hazardLevel) {
    case 3:
      hazardLevelStr = "BAHAYA";
      break;
    case 2:
      hazardLevelStr = "WASPADA";
      break;
    case 1:
    default:
      hazardLevelStr = "NORMAL";
      break;
  }
  lcd.setCursor((20 - hazardLevelStr.length()) / 2, 2);
  lcd.print(hazardLevelStr);

  // Row 4: "KKN STMKG Unit 15" centered
  lcd.setCursor((20 - strlen("KKN STMKG Unit 15")) / 2, 3);
  lcd.print("KKN STMKG Unit 15");
}

void sendTelegramData() {
  String message = "Tinggi Muka Air: " + String(lastTMA) + " cm\n";
  message += "Status Pompa: " + String(lastPumpStatus == 1 ? "ON" : "OFF") + "\n";
  message += "Peringatan Banjir: ";

  if (lastHazardLevel == 3) {
    message += "BAHAYA";
  } else if (lastHazardLevel == 2) {
    message += "WASPADA";
  } else {
    message += "AMAN";
  }

  bot.sendMessage(CHAT_ID, message, "");
}