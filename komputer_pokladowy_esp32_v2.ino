#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h> // Dodano dla stabilności Androida
#include <ESP32Servo.h>
#include <CircularBuffer.hpp>

// --- KONFIGURACJA PINÓW ---
#define I2C_SDA 2
#define I2C_SCL 1
#define LED_RED 5
#define LED_BLUE 6
#define SERVO_PIN 4 // Zmieniono na pin 4

// --- PARAMETRY ---
const float LAUNCH_THRESHOLD_G = 4.5;
const float APOGEE_DROP_THRESHOLD = 0.8;
const unsigned long LOCKOUT_MS = 500;
const byte DNS_PORT = 53;

MPU6050 mpu;
Adafruit_BMP280 bmp;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DNSServer dnsServer; // Obiekt serwera DNS
Servo parachuteServo;

CircularBuffer<float, 10> altBuffer;
CircularBuffer<float, 10> accelBuffer;

enum RocketState { GROUND, CALIBRATING, ARMED, FLIGHT, LANDED };
RocketState currentState = GROUND;

bool parachuteDeployed = false;
float maxAlt = 0;
float currentAlt = 0;
float p0 = 1013.25;
int16_t offsetAz;
unsigned long launchTime = 0;
unsigned long lastMicros = 0;
unsigned long lastLogTime = 0;

// Interfejs HTML (bez zmian w wyglądzie, dodano wsparcie dla Captive Portal)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Rocket Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; background: #121212; color: #e0e0e0; margin: 0; padding: 10px; }
    .card { background: #1e1e1e; padding: 20px; border-radius: 15px; margin: 10px auto; max-width: 400px; border: 1px solid #333; }
    .val { font-size: 50px; color: #00e676; font-weight: bold; margin: 10px 0; }
    .max-alt { color: #ffeb3b; font-size: 20px; }
    .btn { display: block; width: 100%; max-width: 320px; margin: 15px auto; padding: 18px; font-size: 18px; border-radius: 10px; border: none; font-weight: bold; cursor: pointer; text-transform: uppercase; }
    .btn-start { background: #2e7d32; color: white; }
    .btn-deploy { background: #c62828; color: white; height: 160px; font-size: 32px; box-shadow: 0 8px #8e0000; margin-top: 30px; }
    .btn-deploy:active { transform: translateY(4px); box-shadow: 0 4px #8e0000; }
    .btn-stop { background: #f9a825; color: black; margin-top: 30px; }
    .btn-dl { background: #0277bd; color: white; text-decoration: none; display: block; margin: 20px auto; width: 100%; max-width: 320px; padding: 15px; border-radius: 8px; }
    #status { color: #aaa; font-size: 16px; }
  </style>
</head><body>
  <div class="card">
    <div id="status">STATUS: POŁĄCZONY</div>
    <div class="val"><span id="alt">0.0</span><small> m</small></div>
    <div class="max-alt">MAX: <span id="max">0.0</span> m</div>
  </div>
  <button class="btn btn-start" onclick="fetch('/start')">1. Kalibracja i Uzbrojenie</button>
  <button class="btn btn-deploy" onclick="fetch('/deploy')">SPADOCHRON</button>
  <button class="btn btn-stop" onclick="fetch('/stop')">3. Koniec Zapisu</button>
  <a href="/download" class="btn btn-dl">Pobierz Dane CSV</a>
  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    function initWS() {
      websocket = new WebSocket(gateway);
      websocket.onmessage = function(e) {
        var d = JSON.parse(e.data);
        document.getElementById('alt').innerHTML = d.alt;
        document.getElementById('max').innerHTML = d.max;
        document.getElementById('status').innerHTML = "STATUS: " + d.state;
      };
      websocket.onclose = function() { setTimeout(initWS, 2000); };
    }
    window.onload = initWS;
  </script>
</body></html>)rawliteral";

void notify() {
  char json[128];
  const char* st[] = {"NA ZIEMI", "KALIBRACJA", "UZBROJONA", "LOT", "LADOWANIE"};
  sprintf(json, "{\"alt\":\"%.1f\",\"max\":\"%.1f\",\"state\":\"%s\"}", currentAlt, maxAlt, st[currentState]);
  ws.textAll(json);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);

  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(0); 

  if(!SPIFFS.begin(true)) return;
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  bmp.begin(0x76);
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1);

  // WIFI I DNS
  WiFi.softAP("rakieta 192.168.4.1", NULL);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); // Przechwytywanie zapytań DNS
  
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  
  // Captive Portal dla Androida/Samsunga
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *r){ currentState = CALIBRATING; r->send(200); });
  server.on("/deploy", HTTP_GET, [](AsyncWebServerRequest *r){ parachuteServo.write(90); parachuteDeployed = true; r->send(200); });
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *r){ currentState = LANDED; r->send(200); });
  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(SPIFFS, "/data.csv", "text/csv"); });

  server.begin();
}

void loop() {
  dnsServer.processNextRequest(); // Obsługa DNS
  ws.cleanupClients();

  if (currentState == CALIBRATING) {
    digitalWrite(LED_BLUE, HIGH);
    long sAz = 0; float sP = 0;
    for(int i=0; i<200; i++) {
      int16_t ax, ay, az; mpu.getAcceleration(&ax, &ay, &az);
      sAz += az; sP += bmp.readPressure();
      delay(5);
    }
    offsetAz = sAz / 200;
    p0 = (sP / 200.0) / 100.0; 
    File f = SPIFFS.open("/data.csv", "w");
    f.println("TimeMS,Alt,AccZ_G"); f.close();
    maxAlt = 0; parachuteDeployed = false;
    parachuteServo.write(0);
    digitalWrite(LED_BLUE, LOW);
    currentState = ARMED;
  }

  if (currentState == ARMED || currentState == FLIGHT) {
    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0f;
    lastMicros = nowMicros;

    int16_t ax, ay, az; mpu.getAcceleration(&ax, &ay, &az);
    float accZ_G = (float)(az - offsetAz) / 2048.0f; 
    accelBuffer.push(accZ_G);

    if (currentState == ARMED) {
      float avgAcc = 0;
      for(int i=0; i<accelBuffer.size(); i++) avgAcc += accelBuffer[i];
      if ((avgAcc / accelBuffer.size()) > LAUNCH_THRESHOLD_G) {
        currentState = FLIGHT;
        launchTime = millis();
      }
      currentAlt = 0;
    }

    if (currentState == FLIGHT) {
      float rawAlt = bmp.readAltitude(p0);
      altBuffer.push(rawAlt);
      float avgAlt = 0;
      for(int i=0; i<altBuffer.size(); i++) avgAlt += altBuffer[i];
      currentAlt = avgAlt / altBuffer.size();
      if (currentAlt > maxAlt) maxAlt = currentAlt;

      if (!parachuteDeployed && (millis() - launchTime > LOCKOUT_MS)) {
        if (maxAlt - currentAlt > APOGEE_DROP_THRESHOLD && maxAlt > 2.0) {
          parachuteServo.write(90); 
          parachuteDeployed = true;
          digitalWrite(LED_BLUE, HIGH);
        }
      }

      if (millis() - lastLogTime > 40) { 
        File f = SPIFFS.open("/data.csv", "a");
        if(f) {
          f.printf("%lu,%.2f,%.2f\n", millis()-launchTime, currentAlt, accZ_G);
          f.close();
        }
        lastLogTime = millis();
        digitalWrite(LED_BLUE, !digitalRead(LED_BLUE)); 
      }
    }
  }

  static unsigned long lastNotify = 0;
  if (millis() - lastNotify > 250) {
    notify();
    lastNotify = millis();
  }
}