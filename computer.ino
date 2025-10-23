#include <Wire.h>
#include <MPU6050.h>
#include <SPIFFS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <MadgwickAHRS.h>

// I2C pin definitions
#define I2C_SDA 2
#define I2C_SCL 1

// LED pin definitions
#define LED_RED 5
#define LED_BLUE 6

// Buffer settings
#define ALTITUDE_BUFFER_SIZE 10 // Liczba odczytów wysokości do uśredniania
#define VELOCITY_BUFFER_SIZE 5 // Liczba par wysokość-czas do obliczenia prędkości

MPU6050 mpu;
Adafruit_BMP280 bmp;
AsyncWebServer server(80);
Madgwick filter;

// Nagłówek pliku CSV.
const char* csvHeader = "Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure,Altitude,Velocity,Pitch,Roll,Yaw\n";

// Zmienne kalibracyjne
int16_t offsetAx, offsetAy, offsetAz;
float initialPressure;
// float altitude_offset; // Zmienna niepotrzebna po zmianie logiki zerowania wysokości

// Buffers for altitude and time
float altitudeBuffer[ALTITUDE_BUFFER_SIZE];
unsigned long timeBuffer[ALTITUDE_BUFFER_SIZE];
int bufferIndex = 0;

// Zmienne logiki zapisu
unsigned long lastMovementTime = 0;
const unsigned long MOVEMENT_INACTIVITY_TIME = 600000; // 10 minut
const float MOVEMENT_THRESHOLD_ACCEL = 0.5; // [g]
const float MOVEMENT_THRESHOLD_GYRO = 25.0; // [deg/s]
bool isRecording = false;
bool isReadyToLog = false;
unsigned long lastLoopTime = 0;
unsigned long lastSerialPrint = 0;
const unsigned long SERIAL_PRINT_INTERVAL = 1000;

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("Blad inicjalizacji SPIFFS!");
    return;
  }

  if (SPIFFS.exists("/data.csv")) {
    File dataFile = SPIFFS.open("/data.csv", "r");
    size_t fileSize = dataFile.size();
    dataFile.close();

    if (fileSize > strlen(csvHeader)) {
      Serial.println("W pamieci jest juz plik 'data.csv'.");
      Serial.print("Rozmiar pliku: ");
      Serial.print(fileSize);
      Serial.println(" bajtow.");

      Serial.println("Przelaczam w tryb punktu dostepowego (AP) do transferu danych.");
      Serial.println("---");
      Serial.println("1. Polacz sie z siecia Wi-Fi: ESP32-Rocket-Data");
      Serial.println("2. Otworz przegladarke i wejdz na adres: http://192.168.4.1/download");
      Serial.println("3. Po sciagnieciu pliku, otworz: http://192.168.4.1/delete aby go usunac.");
      Serial.println("4. Aby rozpoczac nowy zapis, zrestartuj ESP32.");

      WiFi.softAP("ESP32-Rocket-Data", NULL);

      server.on("/download", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (SPIFFS.exists("/data.csv")) {
          Serial.println("Przesylam plik do pobrania...");
          AsyncWebServerResponse* response = request->beginResponse(SPIFFS, "/data.csv", "text/csv");
          response->addHeader("Content-Disposition", "attachment; filename=data.csv");
          request->send(response);
        } else {
          request->send(404, "text/plain", "Plik nie istnieje!");
        }
      });

      server.on("/delete", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (SPIFFS.exists("/data.csv")) {
          SPIFFS.remove("/data.csv");
          digitalWrite(LED_BLUE, HIGH);
          delay(1000);
          digitalWrite(LED_BLUE, LOW);
          request->send(200, "text/plain", "Plik data.csv zostal usuniety! Mozesz teraz zrestartowac ESP32.");
          Serial.println("Plik data.csv zostal usuniety.");
        } else {
          request->send(404, "text/plain", "Plik nie istnieje!");
          Serial.println("Proba usuniecia pliku, ktory nie istnieje.");
        }
      });

      server.begin();

      while (true) {
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);
      }
    } else {
      SPIFFS.remove("/data.csv");
      Serial.println("Plik 'data.csv' jest pusty. Rozpoczynam nowy zapis.");
    }
  } else {
    Serial.println("Brak pliku 'data.csv' w pamieci. Rozpoczynam nowy zapis.");
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  digitalWrite(LED_RED, HIGH);

  // --- Kalibracja MPU6050 ---
  digitalWrite(LED_BLUE, HIGH);
  mpu.initialize();
  mpu.setSleepEnabled(false);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

  Serial.println("Sprawdzanie polaczenia z MPU6050...");
  if (!mpu.testConnection()) {
    Serial.println("Polaczenie MPU6050 nieudane!");
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
    while (true);
  }

  Serial.println("Trzymaj czujnik nieruchomo. Rozpoczynam kalibracje MPU6050...");
  delay(1000);

  long sumAx = 0, sumAy = 0, sumAz = 0;
  int numReadings = 500;

  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    delay(10);
  }

  offsetAx = sumAx / numReadings;
  offsetAy = sumAy / numReadings;
  offsetAz = sumAz / numReadings;

  Serial.println("Kalibracja MPU6050 zakonczona!");
  Serial.print("Offset A: X=");
  Serial.print(offsetAx);
  Serial.print(", Y=");
  Serial.print(offsetAy);
  Serial.print(", Z=");
  Serial.println(offsetAz);
  Serial.println("---");
  delay(1000);

  // --- Kalibracja BMP280 (poprawiona) ---
  if (!bmp.begin(0x76)) {
    Serial.println("Brak czujnika BMP280 na adresie 0x76!");
    if (!bmp.begin()) {
      Serial.println("Brak czujnika BMP280 na domyslnym adresie!");
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
      while (true);
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  Serial.println("Rozpoczynam kalibracje BMP280...");
  float sumPressure = 0;
  for (int i = 0; i < 50; i++) {
    sumPressure += bmp.readPressure();
    delay(50);
  }
  initialPressure = sumPressure / 50.0; // To jest P0 - ciśnienie na stole
  
  Serial.println("Kalibracja BMP280 zakonczona!");
  Serial.print("Poczatkowe cisnienie (P0): ");
  Serial.print(initialPressure);
  Serial.println(" Pa. Wysokosc zostaje wyzerowana wzgledem tego cisnienia.");
  Serial.println("---");

  digitalWrite(LED_BLUE, LOW);

  File newFile = SPIFFS.open("/data.csv", "w");
  if (newFile) {
    newFile.print(csvHeader);
    newFile.close();
    Serial.println("Utworzono nowy plik data.csv z naglowkiem.");
  }

  Serial.println("Tryb zbierania danych. Czekam na ruch...");
  lastMovementTime = millis();
  isReadyToLog = true;
  lastLoopTime = millis();
}

void loop() {
  if (!isReadyToLog) {
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  // --- Odczyt MPU6050 ---
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  float ax = (float)(ax_raw - offsetAx) / 2048.0;
  float ay = (float)(ay_raw - offsetAy) / 2048.0;
  float az = (float)(az_raw - offsetAz) / 2048.0;

  float gx = (float)gx_raw / 16.4;
  float gy = (float)gy_raw / 16.4;
  float gz = (float)gz_raw / 16.4;

  float accel_norm = sqrt(ax * ax + ay * ay + az * az);
  float gyro_norm = sqrt(gx * gx + gy * gy + gz * gz);

  // --- Odczyt BMP280 i Obliczenia Wolne od Dryftu ---
  float pressure = bmp.readPressure();
  // Wysokość względna: initialPressure jest używane jako ciśnienie P0 na poziomie stołu
  float altitude = bmp.readAltitude(initialPressure); 
  
  // Aktualizacja buforów wysokości i czasu
  altitudeBuffer[bufferIndex] = altitude;
  timeBuffer[bufferIndex] = now;
  bufferIndex = (bufferIndex + 1) % ALTITUDE_BUFFER_SIZE;

  // Obliczanie prędkości (zmiana wysokości / zmiana czasu)
  float velocity = 0;
  if (bufferIndex >= VELOCITY_BUFFER_SIZE) {
      int prevIndex = (bufferIndex - VELOCITY_BUFFER_SIZE + ALTITUDE_BUFFER_SIZE) % ALTITUDE_BUFFER_SIZE;
      float time_delta_s = (float)(now - timeBuffer[prevIndex]) / 1000.0f;
      // Zabezpieczenie przed dzieleniem przez zero
      if (time_delta_s > 0) {
        velocity = (altitude - altitudeBuffer[prevIndex]) / time_delta_s;
      }
  }

  // --- Logika Wykrywania Ruchu ---
  if (abs(accel_norm - 1.0) > MOVEMENT_THRESHOLD_ACCEL || gyro_norm > MOVEMENT_THRESHOLD_GYRO) {
    lastMovementTime = millis();
    if (!isRecording) {
      isRecording = true;
      Serial.println("Ruch wykryty! Rozpoczynam zapis danych.");
    }
  }

  // --- Zapis Danych ---
  if (isRecording) {
    if (millis() % 400 < 200) {
      digitalWrite(LED_BLUE, HIGH);
    } else {
      digitalWrite(LED_BLUE, LOW);
    }

    if (millis() - lastMovementTime < MOVEMENT_INACTIVITY_TIME) {
      filter.updateIMU(gx, gy, gz, ax, ay, -az);
      float pitch = filter.getPitch();
      float roll = filter.getRoll();
      float yaw = filter.getYaw();

      File dataFile = SPIFFS.open("/data.csv", "a");
      if (dataFile) {
        dataFile.print(millis());
        dataFile.print(",");
        dataFile.print(ax);
        dataFile.print(",");
        dataFile.print(ay);
        dataFile.print(",");
        dataFile.print(az);
        dataFile.print(",");
        dataFile.print(gx);
        dataFile.print(",");
        dataFile.print(gy);
        dataFile.print(",");
        dataFile.print(gz);
        dataFile.print(",");
        dataFile.print(pressure);
        dataFile.print(",");
        dataFile.print(altitude);
        dataFile.print(",");
        dataFile.print(velocity);
        dataFile.print(",");
        dataFile.print(pitch);
        dataFile.print(",");
        dataFile.print(roll);
        dataFile.print(",");
        dataFile.println(yaw);
        dataFile.close();
      } else {
        Serial.println("Blad otwarcia pliku CSV!");
      }
      
      if (now - lastSerialPrint > SERIAL_PRINT_INTERVAL) {
        Serial.print("Zapis danych: ");
        Serial.print("Czas: ");
        Serial.print(millis());
        Serial.print(" ms, ");
        Serial.print("Wysokosc: ");
        Serial.print(altitude, 3);
        Serial.print(" m, ");
        Serial.print("Predkosc: ");
        Serial.print(velocity, 3);
        Serial.print(" m/s, ");
        Serial.print("Pitch: ");
        Serial.print(pitch, 1);
        Serial.print(" deg");
        Serial.println();
        lastSerialPrint = now;
      }

    } else {
      isRecording = false;
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
      Serial.println("Brak ruchu przez 10 minut. Zatrzymano zapis danych.");
    }
  } else {
    if (now - lastSerialPrint > SERIAL_PRINT_INTERVAL) {
      Serial.println("Czekam na ruch, aby rozpoczac zapis danych...");
      lastSerialPrint = now;
    }
  }
  delay(10);
}
