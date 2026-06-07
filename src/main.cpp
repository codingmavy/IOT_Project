// ╔══════════════════════════════════════════════════════════════╗
// ║       DOMESTIC ANIMAL MONITORING SYSTEM — CATTLE  v3        ║
// ║  ESP8266 + BMP280 + HC-SR04 x2 + Buzzer + LED               ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>        // ← BMP280 library
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <NewPing.h>

// ── Credentials ───────────────────────────────────────────────
static constexpr char ssid[]     = "POCO X5 Pro 5G";
static constexpr char password[] = "Domain_99";
#define DATABASE_HOST              "iot-project-29231-default-rtdb.firebaseio.com"

// ── Pin Definitions ───────────────────────────────────────────
static constexpr uint8_t  ledPin          = 2;
static constexpr uint8_t  buzzerPin       = 3;

//  HC-SR04 #2 — Food level
static constexpr uint8_t  triggerPin      = 13;
static constexpr uint8_t  echoPin         = 15;

//  HC-SR04 #1 — Cattle presence
#define OBJECT_TRIGGER_PIN                  12
#define OBJECT_ECHO_PIN                     14

static constexpr uint16_t maxDistanceCm   = 200;
static constexpr uint16_t sensorHeightCm  = 50;   // cm from sensor to empty tray bottom

// ── Thresholds ────────────────────────────────────────────────
static constexpr float    CATTLE_DIST_CM  = 80.0; // cattle present if object closer than this
static constexpr int      FOOD_LOW_PCT    = 20;

// ── BMP280 & NTP Config ───────────────────────────────────────
static constexpr uint8_t  bmp280Address        = 0x76;
static constexpr float    seaLevelPressure     = 1013.25f;
static constexpr int      timeOffsetSeconds    = 19800;    // IST UTC+5:30
static constexpr int      timeUpdateIntervalMs = 60000;
static constexpr unsigned long SEND_INTERVAL   = 2000;
static unsigned long      lastSend             = 0;

// ── Visit / Feeding tracking ──────────────────────────────────
static int           visitCount       = 0;
static bool          wasPresent       = false;
static unsigned long presenceStartMs  = 0;
static unsigned long feedingDurSec    = 0;

// Add these variables alongside the existing visit tracking vars
static int foodAtArrival   = 0;
static int foodAtDeparture = 0;
static int foodConsumed    = 0;

// ── Log buffer (last 20 entries) ─────────────────────────────
#define LOG_SIZE 20
static String logBuffer[LOG_SIZE];
static int    logIndex = 0;

// ── Objects ───────────────────────────────────────────────────
WiFiUDP          ntpUDP;
NTPClient        timeClient(ntpUDP, "pool.ntp.org", timeOffsetSeconds, timeUpdateIntervalMs);
Adafruit_BMP280  bmp;                                                        // BMP280
NewPing          sonar(triggerPin, echoPin, maxDistanceCm);                  // Food level
NewPing          objectSonar(OBJECT_TRIGGER_PIN, OBJECT_ECHO_PIN, maxDistanceCm); // Presence
WiFiClientSecure client;

// ── Sensor Data Struct ────────────────────────────────────────
struct SensorReadings {
    float        temperature   = 0.0f;
    int          pressure      = 0;
    int          altitude      = 0;
    int          food_pct      = 0;
    unsigned int objectDistCm  = 0;
    bool         cattlePresent = false;
    String       timestamp;
};

// ════════════════════════════════════════════════════════════════
//  LOG SYSTEM
// ════════════════════════════════════════════════════════════════
static void addLog(const String &msg) {
    String entry = "[" + timeClient.getFormattedTime() + "] " + msg;
    logBuffer[logIndex % LOG_SIZE] = entry;
    logIndex++;
    Serial.println(entry);
}

static void printAllLogs() {
    Serial.println("\n╔══════════ EVENT LOG ══════════╗");
    int total = min(logIndex, LOG_SIZE);
    int start = (logIndex > LOG_SIZE) ? (logIndex % LOG_SIZE) : 0;
    for (int i = 0; i < total; i++) {
        Serial.println("  " + logBuffer[(start + i) % LOG_SIZE]);
    }
    Serial.println("╚═══════════════════════════════╝\n");
}

// ════════════════════════════════════════════════════════════════
//  UTILITY
// ════════════════════════════════════════════════════════════════
static void blinkLed(uint8_t pin, unsigned long intervalMs) {
    digitalWrite(pin, HIGH); delay(intervalMs);
    digitalWrite(pin, LOW);  delay(intervalMs);
}


static void blinkPresence(bool present) {
    if (present) {
        // presence detected
        digitalWrite(ledPin, HIGH); 
    } else {
        // slower pulse for no presence
        blinkLed(ledPin, 250);
    }
}

static void updateFoodBuzzer(bool foodLow) {
    if (foodLow) {
        tone(buzzerPin, 1000);
    } else {
        noTone(buzzerPin);
    }
}

// ════════════════════════════════════════════════════════════════
//  WIFI
// ════════════════════════════════════════════════════════════════
static void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        blinkLed(ledPin, 250);
        Serial.print('.');
    }
    Serial.println("\nConnected: " + WiFi.localIP().toString());
}

void reconnectWiFi() {
    Serial.println("WiFi lost — reconnecting...");
    WiFi.reconnect();
    unsigned long start = millis();
    while (millis() - start < 5000) {
        blinkLed(ledPin, 100);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(ledPin, HIGH);
        Serial.println("\nReconnected!");
    } else {
        digitalWrite(ledPin, LOW);
        Serial.println("\nReconnect failed — trying next loop");
    }
}

// ════════════════════════════════════════════════════════════════
//  BMP280 INIT
// ════════════════════════════════════════════════════════════════
static void configureBmp280() {
    // SDA = GPIO4, SCL = GPIO5
    Wire.begin(4, 5);
    if (!bmp.begin(bmp280Address)) {
        Serial.println(F("BMP280 not found! Check: SDA=GPIO4 SCL=GPIO5"));
        while (true) delay(100);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 ready.");
}

// ════════════════════════════════════════════════════════════════
//  SENSORS
// ════════════════════════════════════════════════════════════════
static SensorReadings gatherSensorData() {
    SensorReadings data;

    data.timestamp   = timeClient.getFormattedTime();

    // BMP280
    data.temperature = bmp.readTemperature();                    // °C
    data.pressure    = bmp.readPressure() / 100;                 // Pa → hPa
    data.altitude    = bmp.readAltitude(seaLevelPressure);       // metres (calc from pressure)

    // Food level — HC-SR04 #2
    unsigned int rawCm = sonar.ping_cm();
    // 0 cm (sensor blocked = tray full) = 100%; sensorHeightCm (empty tray) = 0%
    data.food_pct = constrain(map(rawCm, 0, sensorHeightCm, 100, 0), 0, 100);

    // Cattle presence — HC-SR04 #1
    data.objectDistCm  = objectSonar.ping_cm();
    data.cattlePresent = (data.objectDistCm > 0 && data.objectDistCm < (unsigned int)CATTLE_DIST_CM);

    return data;
}

static void updateVisitTracking(bool present) {
    if (present && !wasPresent) {
        presenceStartMs = millis();
        visitCount++;
        foodAtArrival = gatherSensorData().food_pct;  // snapshot on arrival
        addLog("Cattle ARRIVED — Visit #" + String(visitCount) +
               " | Food: " + String(foodAtArrival) + "%");
    }
    if (!present && wasPresent) {
        feedingDurSec  = (millis() - presenceStartMs) / 1000;
        foodAtDeparture = gatherSensorData().food_pct; // snapshot on departure
        foodConsumed    = foodAtArrival - foodAtDeparture;
        if (foodConsumed < 0) foodConsumed = 0;        // guard sensor noise
        addLog("Cattle LEFT — Duration: " + String(feedingDurSec) +
               "s | Consumed: " + String(foodConsumed) + "%");
    }
    wasPresent = present;
}

static void printSensorData(const SensorReadings &data) {
    Serial.println("── Sensor Data ──────────────");
    Serial.println("Time          : " + data.timestamp);
    Serial.print  ("Temperature   : "); Serial.print(data.temperature, 2); Serial.println(" °C");
    Serial.print  ("Pressure      : "); Serial.print(data.pressure);       Serial.println(" hPa");
    Serial.print  ("Altitude      : "); Serial.print(data.altitude);       Serial.println(" m  (calc from pressure)");
    Serial.print  ("Food Level    : "); Serial.print(data.food_pct);       Serial.println(" %");
    Serial.print  ("Object Dist   : "); Serial.print(data.objectDistCm);   Serial.println(" cm");
    Serial.println("Cattle Present: " + String(data.cattlePresent ? "YES ✓" : "NO"));
    Serial.println("Visit Count   : " + String(visitCount));
    Serial.println("Last Feed Dur : " + String(feedingDurSec) + " s");
    Serial.println("─────────────────────────────");
}

// ════════════════════════════════════════════════════════════════
//  VISIT & FEEDING DURATION TRACKING
// ════════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════════
//  ALERTS
// ════════════════════════════════════════════════════════════════
static void handleAlerts(const SensorReadings &data) {
    if (data.food_pct < FOOD_LOW_PCT) {
        addLog("ALERT: FOOD LOW — " + String(data.food_pct) + "% — REFILL REQUIRED");
    }
}

// ════════════════════════════════════════════════════════════════
//  FIREBASE
// ════════════════════════════════════════════════════════════════
void sendToFirebase(float temp,
                    int   pressure,
                    int   food_pct,
                    int   altitude,
                    unsigned int objectDist,
                    bool  cattlePresent)
{
    
    String json = "{";
    json += "\"temperature\":"        + String(temp, 1)               + ",";
    json += "\"pressure\":"           + String(pressure)              + ",";
    json += "\"altitude\":"           + String(altitude)              + ",";
    json += "\"food\":"               + String(food_pct)              + ",";
    json += "\"object_distance\":"    + String(objectDist)            + ",";
    json += "\"cattle_present\":"     + String(cattlePresent ? 1 : 0) + ",";
    json += "\"visit_count\":"        + String(visitCount)            + ",";
    json += "\"feeding_duration\":"   + String(feedingDurSec)         + ",";
    json += "\"food_at_arrival\":"    + String(foodAtArrival)         + ",";
    json += "\"food_at_departure\":"  + String(foodAtDeparture)       + ",";
    json += "\"food_consumed\":"      + String(foodConsumed)          ;
    json += "}";

    Serial.println("Sending: " + json);

    if (!client.connect(DATABASE_HOST, 443)) {
        Serial.println("Firebase connection failed!");
        addLog("Firebase connection FAILED");
        return;
    }

    client.println("PUT /readings/latest.json HTTP/1.1");
    client.println("Host: " + String(DATABASE_HOST));
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + String(json.length()));
    client.println("Connection: close");
    client.println();
    client.print(json);

    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 5000) {
            Serial.println("Firebase timeout!");
            addLog("Firebase TIMEOUT");
            client.stop();
            return;
        }
    }
    while (client.available()) Serial.write(client.read());
    Serial.println();
    client.stop();
}

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    pinMode(ledPin,    OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    noTone(buzzerPin);

    // 1. WiFi FIRST (same order as your working code)
    connectToWiFi();

    // 2. Firebase SSL
    client.setInsecure();
    Serial.println("Firebase HTTP ready!");

    // 3. NTP
    timeClient.begin();
    timeClient.update();

    // 4. BMP280 (Wire.begin happens inside, after WiFi is up)
    configureBmp280();

    // Startup beep
    tone(buzzerPin, 1000); delay(300); noTone(buzzerPin);

    addLog("System started. IP: " + WiFi.localIP().toString());
}

// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
    digitalWrite(ledPin, HIGH);

    if (WiFi.status() != WL_CONNECTED) {
        reconnectWiFi();
        return;
    }

    timeClient.update();

    if (millis() - lastSend > SEND_INTERVAL) {
        lastSend = millis();

        const SensorReadings data = gatherSensorData();
        printSensorData(data);

        blinkPresence(data.cattlePresent);
        updateFoodBuzzer(data.food_pct < FOOD_LOW_PCT);
        updateVisitTracking(data.cattlePresent);
        handleAlerts(data);

        sendToFirebase(data.temperature,
                       data.pressure,
                       data.food_pct,
                       data.altitude,
                       data.objectDistCm,
                       data.cattlePresent);

        // Auto-print logs every 10 sends (~20 sec)
        if ((lastSend / SEND_INTERVAL) % 10 == 0) {
            printAllLogs();
        }
    }
}

// ════════════════════════════════════════════════════════════════
//  SERIAL COMMAND — type 'L' in Serial Monitor to print logs
// ════════════════════════════════════════════════════════════════
void serialEvent() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'L' || c == 'l') {
            printAllLogs();
        }
    }
}
