#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <NewPing.h>

// ── Credentials ───────────────────────────────────────
static constexpr char ssid[] = "POCO X5 Pro 5G";
static constexpr char password[] = "Domain_99";
#define DATABASE_HOST "iot-project-29231-default-rtdb.firebaseio.com"

// ── Pin Definitions ───────────────────────────────────
static constexpr uint8_t ledPin = 12;
static constexpr uint8_t irPin = 14;
static constexpr uint8_t triggerPin = 13;
static constexpr uint8_t echoPin = 15;
static constexpr uint16_t maxDistanceCm = 200;
static constexpr uint16_t sensorHeightCm = 50;

// ── BMP280 & NTP Config ───────────────────────────────
static constexpr uint8_t bmp280Address = 0x76;
static constexpr float seaLevelPressure = 1013.25f;
static constexpr int timeOffsetSeconds = 19800;
static constexpr int timeUpdateIntervalMs = 60000;

// ── Send Interval ─────────────────────────────────────
static constexpr unsigned long SEND_INTERVAL = 5000;
static unsigned long lastSend = 0;

// ── Objects ───────────────────────────────────────────
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", timeOffsetSeconds, timeUpdateIntervalMs);
Adafruit_BMP280 bmp;
NewPing sonar(triggerPin, echoPin, maxDistanceCm);
WiFiClientSecure client;

// ── Sensor Data Struct ────────────────────────────────
struct SensorReadings
{
    float temperature = 0.0f;
    int pressure = 0;
    int altitude = 0;
    unsigned int distanceCm = 0;
    int rain = 0;
    int food_cm = 0;
    String timestamp;
};

// ════════════════════════════════════════════════════════
// FIREBASE — plain HTTP, no auth, no library
// ════════════════════════════════════════════════════════
void sendToFirebase(float temp,
                    int pressure,
                    int rain,
                    int food_cm,
                    int altitude)
{

    // Build JSON
    String json = "{";
    json += "\"temperature\":" + String(temp, 1) + ",";
    json += "\"pressure\":" + String(pressure) + ",";
    json += "\"rain\":" + String(rain) + ",";
    json += "\"food\":" + String(food_cm) + ",";
    json += "\"altitude\":" + String(altitude);
    json += "}";

    Serial.println("Sending: " + json);

    // Connect
    if (!client.connect(DATABASE_HOST, 443))
    {
        Serial.println("Firebase connection failed!");
        return;
    }

    // HTTP PUT
    client.println("PUT /readings/latest.json HTTP/1.1");
    client.println("Host: " + String(DATABASE_HOST));
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + String(json.length()));
    client.println("Connection: close");
    client.println();
    client.print(json);

    // Wait for response
    unsigned long timeout = millis();
    while (client.available() == 0)
    {
        if (millis() - timeout > 5000)
        {
            Serial.println("Firebase timeout!");
            client.stop();
            return;
        }
    }

    // Print response
    while (client.available())
    {
        Serial.write(client.read());
    }
    Serial.println();
    client.stop();
}

// ════════════════════════════════════════════════════════
// HARDWARE
// ════════════════════════════════════════════════════════
static void blinkLed(uint8_t pin, unsigned long intervalMs)
{
    digitalWrite(pin, HIGH);
    delay(intervalMs);
    digitalWrite(pin, LOW);
    delay(intervalMs);
}

static void initializeHardware()
{
    pinMode(ledPin, OUTPUT);
    pinMode(irPin, INPUT);
}

static void configureBmp280()
{
    if (!bmp.begin(bmp280Address))
    {
        Serial.println(F("BMP280 not found!"));
        while (true)
            delay(100);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
}

// ════════════════════════════════════════════════════════
// WIFI
// ════════════════════════════════════════════════════════
static void connectToWiFi()
{
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        blinkLed(ledPin, 250);
        Serial.print('.');
    }
    Serial.println("\nConnected: " + WiFi.localIP().toString());
}

void reconnectWiFi()
{
    Serial.println("WiFi lost — reconnecting...");
    WiFi.reconnect();
    unsigned long start = millis();
    while (millis() - start < 5000)
    {
        blinkLed(ledPin, 100);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        digitalWrite(ledPin, HIGH);
        Serial.println("\nReconnected!");
    }
    else
    {
        digitalWrite(ledPin, LOW);
        Serial.println("\nReconnect failed — trying next loop");
    }
}

// ════════════════════════════════════════════════════════
// SENSORS
// ════════════════════════════════════════════════════════
static SensorReadings gatherSensorData()
{
    SensorReadings data;
    data.timestamp = timeClient.getFormattedTime();
    data.rain = digitalRead(irPin);
    data.temperature = bmp.readTemperature();
    data.pressure = bmp.readPressure() / 100; // Pa → hPa
    data.altitude = bmp.readAltitude(seaLevelPressure);
    data.distanceCm = sonar.ping_cm();
    data.food_cm = map(data.distanceCm, 0, sensorHeightCm, 0, 100);
    return data;
}

static void printSensorData(const SensorReadings &data)
{
    Serial.println("── Sensor Data ──────────────");
    Serial.println("Time       : " + data.timestamp);
    Serial.println("Rain       : " + String(data.rain));
    Serial.print("Temperature: ");
    Serial.print(data.temperature, 2);
    Serial.println(" °C");
    Serial.print("Pressure   : ");
    Serial.print(data.pressure);
    Serial.println(" hPa");
    Serial.print("Altitude   : ");
    Serial.print(data.altitude);
    Serial.println(" m");
    Serial.print("Food Level : ");
    Serial.print(data.food_cm);
    Serial.println(" %");
    Serial.println("─────────────────────────────");
}

// ════════════════════════════════════════════════════════
// SETUP & LOOP
// ════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);
    initializeHardware();
    connectToWiFi();

    // Firebase SSL
    client.setInsecure();
    Serial.println("Firebase HTTP ready!");


    // NTP
    timeClient.begin();
    timeClient.update();

    // BMP280
    configureBmp280();
}
void loop()
{
    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED)
    {
        reconnectWiFi();
        return;
    }

    timeClient.update();

    if (millis() - lastSend > SEND_INTERVAL)
    {
        lastSend = millis();

        const SensorReadings data = gatherSensorData();
        printSensorData(data);

        sendToFirebase(data.temperature,
                       data.pressure,
                       data.rain,
                       data.food_cm,
                       data.altitude);
    }
}