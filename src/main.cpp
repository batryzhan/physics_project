// ESP32 + MPU6050: измерение скорости удара по мячу.
//
// Датчик крепится на голень/ботинок. Гироскоп измеряет угловую скорость
// ноги w, скорость стопы ~ w * R (R - эффективный радиус вращения),
// скорость мяча ~ v_стопы * K (коэффициент передачи при ударе).
//
// Подключение:  VCC-3V3, GND-GND, SCL-GPIO19, SDA-GPIO18
//
// Протокол (115200 бод, строки):
//   READY                          - прошивка запущена, калибровка завершена
//   D,<ms>,<w_dps>,<a_g>           - живые данные, 50 Гц (макс. за 20 мс)
//   K,<v_foot>,<v_ball>,<peak_dps>,<peak_g>  - зафиксирован удар (м/с)
//   ERR,<текст>                    - ошибка

#include <Arduino.h>
#include <Wire.h>

// ---------- Пины ----------
constexpr int SDA_PIN = 18;
constexpr int SCL_PIN = 19;
constexpr int LED_PIN = 2;  // встроенный светодиод DOIT DevKit

// ---------- Параметры физики (подстрой под себя) ----------
constexpr float EFFECTIVE_RADIUS_M = 0.55f;  // от оси вращения до стопы, м
constexpr float BALL_COEF = 1.25f;           // v_мяча / v_стопы при ударе
constexpr float KICK_START_DPS = 300.0f;     // начало замаха
constexpr float KICK_MIN_PEAK_DPS = 600.0f;  // минимум для засчитанного удара
constexpr float KICK_END_RATIO = 0.60f;      // удар закончен, когда w < 60% пика
constexpr uint32_t KICK_MAX_MS = 500;        // максимум длительности замаха
constexpr uint32_t REFRACTORY_MS = 1200;     // пауза после удара

// ---------- MPU6050 ----------
constexpr float ACCEL_LSB_PER_G = 2048.0f;  // диапазон +-16 g
constexpr float GYRO_LSB_PER_DPS = 16.4f;   // диапазон +-2000 dps
constexpr float DEG2RAD = 0.017453292f;

uint8_t mpuAddr = 0x68;
float gyroBias[3] = {0, 0, 0};

bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

struct Sample {
  float ax, ay, az;  // g
  float gx, gy, gz;  // dps (с вычтенным смещением)
};

bool readRaw(int16_t *v) {  // v[0..6]: ax ay az temp gx gy gz
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, 14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

bool readSample(Sample &s) {
  int16_t r[7];
  if (!readRaw(r)) return false;
  s.ax = r[0] / ACCEL_LSB_PER_G;
  s.ay = r[1] / ACCEL_LSB_PER_G;
  s.az = r[2] / ACCEL_LSB_PER_G;
  s.gx = r[4] / GYRO_LSB_PER_DPS - gyroBias[0];
  s.gy = r[5] / GYRO_LSB_PER_DPS - gyroBias[1];
  s.gz = r[6] / GYRO_LSB_PER_DPS - gyroBias[2];
  return true;
}

void calibrateGyro() {
  const int N = 1500;
  double sum[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < N; i++) {
    int16_t r[7];
    if (readRaw(r)) {
      sum[0] += r[4] / GYRO_LSB_PER_DPS;
      sum[1] += r[5] / GYRO_LSB_PER_DPS;
      sum[2] += r[6] / GYRO_LSB_PER_DPS;
      n++;
    }
    delay(1);
  }
  if (n > 0)
    for (int k = 0; k < 3; k++) gyroBias[k] = (float)(sum[k] / n);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  // MPU6050 может быть на 0x68 (AD0=GND/свободен) или 0x69 (AD0=3V3)
  while (true) {
    if (mpuPresent(0x68)) { mpuAddr = 0x68; break; }
    if (mpuPresent(0x69)) { mpuAddr = 0x69; break; }
    Serial.println("ERR,MPU6050 not found (check SDA=18 SCL=19 3V3 GND)");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }

  i2cWrite(0x6B, 0x01);  // выйти из sleep, тактирование от гироскопа X
  delay(100);
  i2cWrite(0x1A, 0x01);  // DLPF ~188 Гц, выход гироскопа 1 кГц
  i2cWrite(0x19, 0x00);  // делитель 0 -> 1000 Гц
  i2cWrite(0x1B, 0x18);  // гироскоп +-2000 dps
  i2cWrite(0x1C, 0x18);  // акселерометр +-16 g

  digitalWrite(LED_PIN, HIGH);  // горит = калибровка, держи датчик неподвижно
  calibrateGyro();
  digitalWrite(LED_PIN, LOW);

  Serial.println("READY");
}

enum State { IDLE, SWING, COOLDOWN };

void loop() {
  static uint32_t nextUs = micros();
  static State state = IDLE;
  static float peakW = 0, peakG = 0;
  static uint32_t swingStart = 0, cooldownStart = 0;
  static float winW = 0, winG = 0;  // максимумы за окно вывода
  static uint32_t lastPrint = 0;

  // темп 1 кГц
  uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;
  nextUs += 1000;
  if ((int32_t)(now - nextUs) > 5000) nextUs = now + 1000;  // догоняем без лавины

  Sample s;
  if (!readSample(s)) return;

  float w = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);  // dps
  float g = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);  // g
  uint32_t ms = millis();

  if (w > winW) winW = w;
  if (g > winG) winG = g;
  if (ms - lastPrint >= 20) {
    Serial.printf("D,%lu,%.0f,%.2f\n", (unsigned long)ms, winW, winG);
    winW = 0;
    winG = 0;
    lastPrint = ms;
  }

  switch (state) {
    case IDLE:
      if (w > KICK_START_DPS) {
        state = SWING;
        swingStart = ms;
        peakW = w;
        peakG = g;
      }
      break;

    case SWING: {
      if (w > peakW) peakW = w;
      if (g > peakG) peakG = g;
      bool ended = (w < peakW * KICK_END_RATIO) || (ms - swingStart > KICK_MAX_MS);
      if (!ended) break;

      if (peakW >= KICK_MIN_PEAK_DPS) {
        float vFoot = peakW * DEG2RAD * EFFECTIVE_RADIUS_M;
        float vBall = vFoot * BALL_COEF;
        Serial.printf("K,%.2f,%.2f,%.0f,%.1f\n", vFoot, vBall, peakW, peakG);
        digitalWrite(LED_PIN, HIGH);
        state = COOLDOWN;
        cooldownStart = ms;
      } else {
        state = IDLE;  // слабое движение, не удар
      }
      break;
    }

    case COOLDOWN:
      if (ms - cooldownStart > 150) digitalWrite(LED_PIN, LOW);
      if (ms - cooldownStart > REFRACTORY_MS && w < KICK_START_DPS) state = IDLE;
      break;
  }
}
